from config import TYPE_CONTROLLER, TYPE_CONFIGURATION, get_required_configurable_entity_by_name, TYPE_EXPORTER, TYPE_TAG_PROCESSOR, TYPE_RFID_READER, get_entities_by_type
from config.configuration import default_configuration, Configuration
from controllers.controller import Controller
from tag.tag_types import TagType
from tag.tag_processor import TagProcessor
from tag.mifare_classic_tag_processor import MifareClassicTagProcessor
from tag.mifare_ultralight_tag_processor import MifareUltralightTagProcessor
from reader.mifare_classic_reader import MifareClassicReader
from reader.mifare_ultralight_reader import MifareUltralightReader
from reader.rfid_reader import RfidReader
from exporters.exporter import Exporter, ExporterEvent
from reader.scan_result import ScanResult
from filament import GenericFilament
from typing import cast
import time
import logging
import threading

class Runtime:
    def __init__(self):
        configs = cast(list[Configuration], get_entities_by_type(TYPE_CONFIGURATION))

        if len(configs) == 0:
            configs = [default_configuration()]
        elif len(configs) >= 2:
            logging.warning(f"Multiple configurations found, using the first one: {[config.name for config in configs]}")

        self.config = configs[0]

        self.rfid_readers : list[RfidReader] = [x for x in cast(list[RfidReader], get_entities_by_type(TYPE_RFID_READER)) if x.enabled]
        self.tag_processors : list[TagProcessor] = [x for x in cast(list[TagProcessor], get_entities_by_type(TYPE_TAG_PROCESSOR)) if x.enabled]
        self.exporters : list[Exporter] = [x for x in cast(list[Exporter], get_entities_by_type(TYPE_EXPORTER)) if x.enabled]
        self.controllers : list[Controller] = [x for x in cast(list[Controller], get_entities_by_type(TYPE_CONTROLLER)) if x.enabled]

        logging.debug(f"Tag processors: {','.join([processor.name for processor in self.tag_processors])}")
        logging.debug(f"Exporters: {','.join([exporter.name for exporter in self.exporters])}")

        for controller in self.controllers:
            controller.runtime = self # type: ignore

        self.mifare_classic_processors = [processor for processor in self.tag_processors if isinstance(processor, MifareClassicTagProcessor)]
        self.mifare_ultralight_processors = [processor for processor in self.tag_processors if isinstance(processor, MifareUltralightTagProcessor)]

        self.read_retries_left = [0] * len(self.rfid_readers)

        # Serialises reader access between the scan loop and a write request.
        # Both run on their own threads (the loop on the main thread, a write
        # arriving on a controller thread), and the FM175xx is one shared SPI
        # bus, so concurrent access would corrupt both.
        self.reader_lock = threading.Lock()

    def _notify_exporters(self, scan: ScanResult|None, filament: GenericFilament|None, reader: RfidReader, event: ExporterEvent):
        for exporter in self.exporters:
            if exporter.has_event(event):
                exporter.export_data(scan, filament, reader)

    def start_reading_tag(self, slot: int):
        if slot < 0 or slot >= len(self.rfid_readers):
            logging.error(f"Invalid slot number: {slot}")
            return
        
        self.read_retries_left[slot] = self.config.read_retries
        logging.info(f"Received request to read tag on slot {slot} with {self.config.read_retries} retries")

    def write_ntag(self, slot: int, start_page: int, data: bytes) -> dict:
        """Program an NTAG/Ultralight tag on a slot's reader, holding the same
        lock the scan loop uses so the two never touch the bus at once.

        Returns a plain dict so a controller can serialise it straight to a
        result file. A write is only attempted after a scan confirms a tag is
        present and is an Ultralight-family tag; the reader stack then writes
        and reads back before reporting success."""
        if slot < 0 or slot >= len(self.rfid_readers):
            return {"ok": False, "error": f"invalid slot {slot}"}

        reader = self.rfid_readers[slot]
        if not isinstance(reader, MifareUltralightReader):
            return {"ok": False, "error": f"reader on slot {slot} cannot write NTAG"}

        with self.reader_lock:
            reader.start_session()
            try:
                scan_result = reader.scan()
                if scan_result is None:
                    return {"ok": False, "error": "no tag in the reader's field"}

                uid = scan_result.uid.hex()
                if scan_result.tag_type != TagType.MifareUltralight:
                    return {"ok": False, "uid": uid,
                            "error": "not an NTAG/Ultralight tag (a Classic tag "
                                     "cannot be programmed this way)"}

                ok = reader.write_mifare_ultralight(scan_result, start_page, data)
            finally:
                reader.end_session()

        if ok:
            return {"ok": True, "uid": uid, "pages": len(data) // 4}
        return {"ok": False, "uid": uid,
                "error": "the reader reported the write failed (see openrfid log)"}

    def loop(self):
        while True:
            for i, reader in enumerate(self.rfid_readers):
                if not self.config.auto_read_mode and self.read_retries_left[i] <= 0:
                    continue

                logging.debug(f"Processing reader {reader.name}, retries left: {self.read_retries_left[i]}")
                try:
                    with self.reader_lock:
                        scan_result, filament, retry = self.process_reader_single(reader)
                except Exception as e:
                    logging.exception(f"Error processing reader {reader.name}: {e}")
                    scan_result, filament, retry = None, None, False # Assume exceptions are not transient

                if retry and self.read_retries_left[i] <= 1:
                    retry = False # Don't allow retry if this is the last retry left

                if retry:
                    self.read_retries_left[i] -= 1
                    logging.info(f"No tag detected on reader {reader.name}, will retry, retries left: {self.read_retries_left[i]}")
                    continue
                
                if filament:
                    logging.info(f"Successfully read tag with UID {scan_result.uid.hex().upper()} on reader {reader.name}")
                    self._notify_exporters(scan_result, filament, reader, ExporterEvent.TAG_READ)
                    self.read_retries_left[i] = 0
                elif scan_result:
                    logging.info(f"Detected tag with UID {scan_result.uid.hex().upper()} on reader {reader.name} but failed to read data")
                    self._notify_exporters(scan_result, None, reader, ExporterEvent.TAG_PARSE_ERROR)
                    self.read_retries_left[i] = 0
                elif self.config.auto_read_mode: # Auto-mode
                    # TODO: if tag was previously detected, it should at least once notify exporters about tag not present
                    logging.info(f"No tag detected on reader {reader.name}, but auto read mode is enabled, will retry")
                else: # Fatal non-retrable error
                    logging.info(f"No tag detected on reader {reader.name}")
                    self._notify_exporters(None, None, reader, ExporterEvent.TAG_NOT_PRESENT)
                    self.read_retries_left[i] = 0

            time.sleep(self.config.read_interval_seconds)

    def process_reader_single(self, reader: RfidReader) -> tuple[ScanResult|None, GenericFilament|None, bool]:
        reader.start_session()
        scan_result = reader.scan()
        reader.end_session()
        if scan_result is None:
            logging.debug("No tag detected")
            return None, None, True
        
        uid = scan_result.uid.hex()
        
        if self.config.auto_read_mode and reader.is_same_tag(uid):
            logging.debug("Same tag detected as last read, skipping processing")
            return None, None, False
        
        logging.info(scan_result.pretty_text())

        filament = None
        retry = False
        if scan_result.tag_type == TagType.MifareClassic1k and isinstance(reader, MifareClassicReader):
            filament, retry = self.process_mifare_classic(reader, scan_result)
        elif scan_result.tag_type == TagType.MifareUltralight and isinstance(reader, MifareUltralightReader):
            filament, retry = self.process_mifare_ultralight(reader, scan_result)

        if filament:
            logging.info(filament.pretty_text())
            reader.set_last_read_uid(uid)
        else:
            logging.warning(f"Failed to read data from tag.")

        return (scan_result, filament, retry)

    def process_mifare_classic(self, reader: MifareClassicReader, scan_result: ScanResult) -> tuple[GenericFilament | None, bool]:
        any_retryable = False

        for processor in self.mifare_classic_processors:
            reader.start_session()

            if reader.scan() == None:
                logging.warning("Tag lost before reading")
                reader.end_session()
                return None, True

            logging.debug(f"Attempting to read with processor: {processor.name}")
            auth = processor.authenticate_tag(scan_result)

            if auth is None:
                logging.warning("Authentication failed with processor, skipping processor")
                reader.end_session()
                continue

            card_data, retryable = reader.read_mifare_classic(scan_result, auth)
            reader.end_session()

            if card_data is not None:
                logging.debug(f"Read MIFARE Classic card data: {card_data.hex().upper()}")
                return processor.process_tag(scan_result, card_data), False

            logging.warning("Failed to read MIFARE Classic card data")
            if retryable:
                any_retryable = True

        return None, any_retryable
    
    def process_mifare_ultralight(self, reader: MifareUltralightReader, scan_result: ScanResult) -> tuple[GenericFilament | None, bool]:
        reader.start_session()

        if reader.scan() == None:
            logging.warning("Tag lost before reading")
            reader.end_session()
            return None, True

        card_data = reader.read_mifare_ultralight(scan_result)
        reader.end_session()

        if card_data is not None:
            logging.debug(f"Read MIFARE Ultralight card data: {card_data.hex().upper()}")

            for processor in self.mifare_ultralight_processors:
                logging.debug(f"Attempting to read with processor: {processor.name}")
                filament = processor.process_tag(scan_result, card_data)
                if filament is not None:
                    return filament, False

            return None, False
        else:
            logging.warning("Failed to read MIFARE Ultralight card data")
            return None, True