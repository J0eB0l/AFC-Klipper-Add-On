from abc import abstractmethod
from reader.rfid_reader import RfidReader
from reader.scan_result import ScanResult

class MifareUltralightReader(RfidReader):
    def __init__(self, config: dict):
        super().__init__(config)

    @abstractmethod
    def read_mifare_ultralight(self, scan_result : ScanResult) -> bytes|None:
        """Reads data from a Mifare Ultralight tag."""
        raise NotImplementedError("Subclasses must implement this method")

    def write_mifare_ultralight(self, scan_result : ScanResult, start_page : int, data : bytes) -> bool:
        """Writes data to a Mifare Ultralight tag, starting at start_page.

        Not abstract on purpose: a reader that cannot write inherits this and
        reports so, rather than every existing implementation becoming
        uninstantiable. data must be a whole number of 4-byte pages."""
        return False