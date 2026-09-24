# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Writing blank NTAG stickers, for every unit that has a host-side reader.
#
# The reader stack is transport-agnostic, so any reader with a reg_read/reg_write
# link can program a tag. Each RFID unit registers its readers here at ready and
# this module owns the commands:
#
#   AFC_RFID_READERS                list what is available and which are online
#   AFC_RFID_WRITE READER=<name>    write a tag on one of them
#
# There is no config section. The registry lives on the printer object, and the
# first unit to register also registers the commands.
#
# NTAG only: Bambu and Snapmaker tags are MIFARE Classic behind per-tag derived
# keys and are refused by name.

from __future__ import annotations
import threading
import chelper
from typing import Any, Callable, Dict, List, Optional, Tuple

from extras.AFC_RFID import _cached_spoolman_client
from extras.AFC_rfid_readers import (ANYCUBIC_IMAGE_START,
                                     CLASSIC_DEFAULT_KEY,
                                     classic_write_block, decode_afc_block,
                                     decode_anycubic, encode_tag_payload,
                                     read_tag, write_tag)

#: How long a write may take before the g-code gives up. Every page write and
#: read-back is a round trip over the ACE2's serial passthrough.
WRITE_TIMEOUT_S = 90.0


class StageError(RuntimeError):
    """
    A unit's stage hook will not move the filament for this write (the lane is
    in the toolhead, say). The command reports it and writes nothing.
    """


class RfidWriteTarget:
    """One writable reader, as registered by its unit."""

    def __init__(self, name: str, label: str, unit: Any,
                 open_link: Callable[[], Any],
                 prepare: Optional[Callable[[Any], None]] = None,
                 release: Optional[Callable[[Any], None]] = None,
                 threaded: bool = False,
                 write_payload: Optional[
                     Callable[[bytes], Tuple[Optional[str],
                                             Optional[str]]]] = None,
                 stage: Optional[Callable[[str], Any]] = None,
                 unstage: Optional[Callable[[Any], None]] = None,
                 exclude: Optional[Callable[[Any], Optional[
                     Callable[[str], bool]]]] = None,
                 stage_around: Optional[Callable[[str, Callable[..., Any]],
                                                 Any]] = None,
                 serves: Optional[Callable[[str], bool]] = None) -> None:
        """
        A reader the write command can target.

        :param name: unique key, "<unit kind>:<reader>", e.g. "bt:reader0"
        :param label: human description for AFC_RFID_READERS
        :param unit: owning unit, providing .afc, .reactor and .logger
        :param open_link: returns a live reg_read/reg_write link, or None when
            the reader is offline. Ignored when write_payload is given.
        :param prepare: run before the write, to take the reader off whatever
            else owns it. Defaults to reader_power(True) when the link has it.
        :param release: run after the write, always, even on failure. Defaults
            to reader_power(False) when the link has it.
        :param threaded: whether the write may run on a worker thread. Only
            for a transport the host owns outright; a link through a Klipper MCU
            must run on the reactor or Klipper shuts down. Default false.
        :param write_payload: for a reader this host cannot drive directly (the
            U1's, owned by the OpenRFID daemon), a function taking the encoded
            tag bytes and returning (uid, error). Replaces the whole
            open_link/prepare/write_tag/release path; a blocking hand-off wants
            threaded=True.
        :param stage: run on the reactor before the write, given the lane the
            tag belongs to, to bring the tag into the reader's field and hold
            it there. Returns a token passed to unstage. None means the tag is
            already in range (a stationary reader).
        :param unstage: run on the reactor after the write, always, given the
            token stage returned, to put the filament back.
        :param exclude: given the token stage returned, a uid_hex -> bool test
            for tags the write must pass over, such as the other spool's tag on
            a shared antenna.
        :param stage_around: for a unit whose pull-in scan cannot be split into
            stage and unstage (ACE 2, OpenAMS): run on the reactor as
            stage_around(lane, body). It calls body(is_excluded) once with the
            tag stopped in the field, restores the filament and returns what
            body returned, or raises StageError when the tag never came to rest.
            Used instead of stage/unstage when given.
        :param serves: lane name -> whether this reader is the one that lane's
            spool passes, so LANE= alone can pick the reader
        """
        self.name = name
        self.label = label
        self.unit = unit
        self.open_link = open_link
        self.prepare = prepare or _default_prepare
        self.release = release or _default_release
        self.threaded = threaded
        self.write_payload = write_payload
        self.stage = stage
        self.unstage = unstage
        self.exclude = exclude
        self.stage_around = stage_around
        self.serves = serves


def _default_prepare(link: Any) -> None:
    """
    Hand the reader to the host, where the transport allows it.

    reader_power is optional in the link contract, so this checks for it.

    :param link: the reader link
    """
    power = getattr(link, "reader_power", None)
    if power is not None:
        power(True)


def _default_release(link: Any) -> None:
    """
    Give the reader back. Best-effort: a failure here must not mask the write's
    own result.

    :param link: the reader link
    """
    power = getattr(link, "reader_power", None)
    if power is not None:
        try:
            power(False)
        except Exception:
            pass


def _registry(printer: Any) -> Dict[str, RfidWriteTarget]:
    """
    The printer's reader registry, created on first use.

    Kept on the printer rather than at module scope, since Klipper can build a
    new Printer without re-importing extras and module state would go stale.

    :param printer: the Klipper printer object
    :return dict: name -> target, in registration order
    """
    reg = getattr(printer, "_afc_rfid_write_registry", None)
    if reg is None:
        reg = {}
        printer._afc_rfid_write_registry = reg
    return reg


def register_reader(printer: Any, name: str, label: str, unit: Any,
                    open_link: Callable[[], Any],
                    prepare: Optional[Callable[[Any], None]] = None,
                    release: Optional[Callable[[Any], None]] = None,
                    threaded: bool = False,
                    write_payload: Optional[
                        Callable[[bytes], Tuple[Optional[str],
                                                Optional[str]]]] = None,
                    stage: Optional[Callable[[str], Any]] = None,
                    unstage: Optional[Callable[[Any], None]] = None,
                    exclude: Optional[Callable[[Any], Optional[
                        Callable[[str], bool]]]] = None,
                    stage_around: Optional[Callable[
                        [str, Callable[..., Any]], Any]] = None,
                    serves: Optional[Callable[[str], bool]] = None) -> None:
    """
    Register one reader as a write target, and ensure the commands exist.

    Safe to call again for the same name (a unit that re-probes on reconnect
    does): the entry is replaced, not duplicated.

    :param printer: the Klipper printer object
    :param name: unique key, "<unit kind>:<reader>"
    :param label: human description for AFC_RFID_READERS
    :param unit: owning unit, providing .afc, .reactor and .logger
    :param open_link: returns a live link, or None when the reader is offline
    :param prepare: run before the write, to take the reader off whatever else
        owns it
    :param release: run after the write, always
    :param threaded: whether the write may run off the reactor; only for a
        transport the host owns outright (see RfidWriteTarget)
    :param write_payload: a payload -> (uid, error) hand-off for a reader this
        host cannot drive itself (see RfidWriteTarget)
    :param stage: reactor-side hook to bring the tag into range for the lane,
        for a reader that must spin the spool (see RfidWriteTarget)
    :param unstage: reactor-side hook to restore the filament afterwards
    :param exclude: given the stage token, a test for tags the write must pass
        over (see RfidWriteTarget)
    :param stage_around: reactor-side wrapper that runs the unit's own pull-in
        scan around the write (see RfidWriteTarget)
    :param serves: lane name -> whether this reader serves that lane
    """
    _registry(printer)[name] = RfidWriteTarget(
        name, label, unit, open_link, prepare, release, threaded,
        write_payload, stage, unstage, exclude, stage_around, serves)
    _ensure_commands(printer)


def _ensure_commands(printer: Any) -> None:
    """
    Register the g-code commands once per printer.

    :param printer: the Klipper printer object
    """
    if getattr(printer, "_afc_rfid_write_cmds", False):
        return
    gcode = printer.lookup_object("gcode", None)
    if gcode is None:
        # Leave the flag clear so the next unit to register tries again.
        return
    printer._afc_rfid_write_cmds = True
    gcode.register_command(
        "AFC_RFID_READERS", lambda gcmd: cmd_AFC_RFID_READERS(printer, gcmd),
        desc="List every RFID reader that can write a tag, and which are "
             "online. AFC_RFID_READERS")
    gcode.register_command(
        "AFC_RFID_WRITE", lambda gcmd: cmd_AFC_RFID_WRITE(printer, gcmd),
        desc="Write a blank NTAG sticker so a third-party spool reads like a "
             "branded one. AFC_RFID_WRITE READER=<name>|LANE=<lane> "
             "[SPOOL=<id>] [TYPE=] "
             "[BRAND=] [SKU=] [COLOR=RRGGBB] [WEIGHT=] [DIAMETER=] [DENSITY=] "
             "[HOTEND_MIN=] [HOTEND_MAX=] [BED=]")
    gcode.register_command(
        "AFC_RFID_READ", lambda gcmd: cmd_AFC_RFID_READ(printer, gcmd),
        desc="Read the tag at a reader and report it, the quick way to "
             "confirm a write. AFC_RFID_READ READER=<name>")
    gcode.register_command(
        "AFC_RFID_CLASSIC_WRITE",
        lambda gcmd: cmd_AFC_RFID_CLASSIC_WRITE(printer, gcmd),
        desc="Write one 16-byte MIFARE Classic block (blank/magic cards). "
             "AFC_RFID_CLASSIC_WRITE READER=<name> BLOCK=<n> DATA=<32hex> "
             "[KEY=<12hex>]")
    gcode.register_command(
        "AFC_RFID_ERASE", lambda gcmd: cmd_AFC_RFID_ERASE(printer, gcmd),
        desc="Erase the tag at a reader: zero its record so it reads blank "
             "again. AFC_RFID_ERASE READER=<name>")
    gcode.register_command(
        "AFC_RFID_ENROLL", lambda gcmd: cmd_AFC_RFID_ENROLL(printer, gcmd),
        desc="Enroll a blank tag into Spoolman: create or link a spool, write "
             "the tag, bind its UID. AFC_RFID_ENROLL READER=<name>|LANE=<lane> "
             "[SPOOL=<id>] "
             "[TYPE=] [BRAND=] [COLOR=RRGGBB] [WEIGHT=] [DIAMETER=] [DENSITY=] "
             "[HOTEND_MIN=] [HOTEND_MAX=] [BED=] [DRY_TEMP=] [DRY_TIME=] [SKU=]")


def _online(target: RfidWriteTarget) -> bool:
    """
    Whether a target can currently be opened.

    :param target: the reader to check
    :return bool: True when a link comes back
    """
    if target.write_payload is not None:
        # A hand-off target has no link to open; it counts as online because it
        # is registered. Whether its daemon answers only shows on a write.
        return True
    try:
        return target.open_link() is not None
    except Exception:
        return False


def resolve_reader(printer: Any, name: Optional[str],
                   lane: Optional[str] = None
                   ) -> Tuple[Optional[RfidWriteTarget], Optional[str]]:
    """
    Find the reader a READER= value names.

    A bare reader name ("reader0") is accepted when only one unit has one by
    that name, so the qualified form is only needed to break a tie. With no
    name at all, a single registered reader is used, or the one reader that
    serves ``lane``; anything more is ambiguous and reported as an error.

    :param printer: the Klipper printer object
    :param name: the READER= value, or None
    :param lane: the LANE= value, or None
    :return tuple: (target, error message)
    """
    reg = _registry(printer)
    if not reg:
        return None, ("no RFID readers are registered. AFC_RFID_WRITE needs a "
                      "unit with a host-side reader (BoxTurtle, ACE2, ViViD "
                      "or OpenAMS).")
    names = ", ".join(reg)
    if not name:
        if len(reg) == 1:
            return next(iter(reg.values())), None
        if lane:
            hits = [t for t in reg.values()
                    if t.serves is not None and _serves(t, lane)]
            if len(hits) == 1:
                return hits[0], None
            if not hits:
                return None, (f"no reader serves {lane}; give READER=. "
                              f"Available: {names}")
        return None, f"READER= is required. Available: {names}"
    want = name.strip().lower()
    for key, target in reg.items():
        if key.lower() == want:
            return target, None
    # Bare reader name, unqualified by unit.
    hits = [t for k, t in reg.items() if k.split(":")[-1].lower() == want]
    if len(hits) == 1:
        return hits[0], None
    if len(hits) > 1:
        both = ", ".join(t.name for t in hits)
        return None, f"{name} is ambiguous, name the unit too: {both}"
    return None, f"no reader called {name}. Available: {names}"


def _serves(target: RfidWriteTarget, lane: str) -> bool:
    """
    Whether a target's reader serves a lane; a failing check says no.

    :param target: the reader
    :param lane: the lane name
    :return bool: True when the unit says the lane's spool passes this reader
    """
    try:
        return bool(target.serves(lane))
    except Exception:
        return False


def spool_fields(unit: Any, spool_id: int) -> Tuple[Dict[str, Any], str]:
    """
    Read a Spoolman spool and map it to tag fields. Worker thread only.

    :param unit: the owning unit, for its .afc
    :param spool_id: Spoolman spool id
    :return tuple: (fields, note); fields is empty when the spool cannot be
        read, with the note saying why
    """
    client = _cached_spoolman_client(getattr(unit, "afc", None))
    if client is None:
        return {}, "no Spoolman client"
    try:
        sp = client.get_spool(spool_id)
    except Exception as e:
        return {}, f"Spoolman read failed: {e}"
    if not isinstance(sp, dict):
        return {}, f"Spoolman has no spool {spool_id}"
    fil = sp.get("filament") or {}
    # The full spool weight, not what is left: Spoolman tracks remaining weight
    # against the spool id the tag also carries.
    weight = (fil.get("weight") or sp.get("initial_weight")
              or sp.get("remaining_weight"))
    out: Dict[str, Any] = {"spool_id": spool_id}
    vendor = (fil.get("vendor") or {}).get("name")
    if vendor:
        out["manufacturer"] = vendor
    if fil.get("material"):
        out["ftype"] = fil["material"]
    if fil.get("name"):
        out["sku"] = fil["name"]
    if fil.get("color_hex"):
        out["color_argb"] = 0xFF000000 | (
            int(str(fil["color_hex"]).lstrip("#")[:6], 16) & 0xFFFFFF)
    if fil.get("diameter"):
        out["diameter_mm"] = float(fil["diameter"])
    if fil.get("density"):
        out["density"] = float(fil["density"])
    if weight:
        out["weight_g"] = int(round(float(weight)))
    if fil.get("settings_extruder_temp"):
        out["hotend_max_c"] = int(fil["settings_extruder_temp"])
    if fil.get("settings_bed_temp"):
        out["bed_temp_c"] = int(fil["settings_bed_temp"])
    return out, f"spool {spool_id}"


def _write_blocking(target: RfidWriteTarget, spool_id: int,
                    overrides: Dict[str, Any],
                    is_excluded: Optional[Callable[[str], bool]] = None
                    ) -> Tuple[Any, ...]:
    """
    Fetch, encode, write, verify and bind. Worker thread only.

    :param target: the reader holding the blank tag
    :param spool_id: Spoolman spool id to source fields from, 0 for none
    :param overrides: explicit g-code params, which win over Spoolman
    :param is_excluded: uid_hex -> bool, tags in the field to pass over
    :return tuple: (uid, error, fields, note)
    """
    fields: Dict[str, Any] = {}
    note = ""
    if spool_id:
        fields, note = spool_fields(target.unit, spool_id)
        if not fields:
            return None, note, {}, ""
    fields.update(overrides)
    payload = encode_tag_payload(**fields)
    if target.write_payload is not None:
        # A reader this host does not drive (the U1's OpenRFID daemon): hand
        # off the bytes and take back its verdict.
        uid, err = target.write_payload(payload)
    else:
        link = target.open_link()
        if link is None:
            return None, f"{target.name} is offline", {}, ""
        target.prepare(link)
        try:
            uid, err = write_tag(link, payload,
                                     is_excluded=is_excluded)
        finally:
            target.release(link)
    # Bind the tag's factory UID to the spool so the next scan resolves it.
    if not err and spool_id and uid:
        try:
            _cached_spoolman_client(
                getattr(target.unit, "afc", None)).write_spool_metadata(
                    spool_id, uid=uid)
            note += ", uid bound"
        except Exception as e:
            note += f", but binding the uid failed: {e}"
    return uid, err, fields, note


def _with_staging(target: RfidWriteTarget, lane: Optional[str],
                  body: Callable[[Optional[Callable[[str], bool]]],
                                 Tuple[Any, ...]]) -> Tuple[Any, ...]:
    """
    Run ``body`` with the tag brought into the reader's field for ``lane``.

    stage/unstage move filament and must run on the reactor; only ``body``
    may go to a worker. A unit whose scan is one piece of motion registers
    stage_around instead and calls body itself while the scan holds the tag.
    A stationary reader, or a call with no lane, runs body directly.

    :param target: the write target
    :param lane: the lane whose spool carries the tag, or None
    :param body: the write to run once the tag is in range, given the tags
        to pass over (the target's exclude for this staging, or None)
    :return tuple: whatever body returns
    """
    if lane and target.stage_around is not None:
        result: List[Any] = []

        def _once(excluded: Optional[Callable[[str], bool]] = None) -> Any:
            """
            Run the write where the unit's scan holds the tag, once only.

            :param excluded: uid_hex -> bool test for tags to pass over
            :return Any: what body returned
            """
            if not result:
                result.append(body(excluded))
            return result[0]

        target.stage_around(lane, _once)
        if not result:
            error_str = (f"{lane}'s tag did not come to rest in the reader's "
                         f"field, so nothing was written")
            raise StageError(error_str)
        return result[0]
    token = target.stage(lane) if (lane and target.stage is not None) else None
    try:
        excluded = (target.exclude(token)
                    if token is not None and target.exclude is not None
                    else None)
        return body(excluded)
    finally:
        if token is not None and target.unstage is not None:
            target.unstage(token)


def run_write(target: RfidWriteTarget, spool_id: int,
              overrides: Dict[str, Any],
              lane: Optional[str] = None) -> Tuple[Any, ...]:
    """
    Run a write, on a worker thread where the transport allows it.

    Only a transport the host owns outright may go off the reactor; anything
    through a Klipper MCU must stay on it or Klipper shuts down. The unit says
    which at registration.

    :param target: the reader holding the blank tag
    :param spool_id: Spoolman spool id to source fields from, 0 for none
    :param overrides: explicit g-code params
    :param lane: the lane whose spool holds the tag, for a reader that must spin
        it into range; None when the tag is already at the coil
    :return tuple: (uid, error, fields, note)
    """
    def _body(excluded: Optional[Callable[[str], bool]]) -> Tuple[Any, ...]:
        """
        Run the blocking write, on a worker thread when the target allows it.

        :param excluded: uid_hex -> bool test for tags to pass over
        :return tuple: (uid, error, fields, note)
        """
        if not target.threaded:
            return _write_blocking(target, spool_id, overrides, excluded)
        reactor = target.unit.reactor
        completion = reactor.completion()
        result: List[Any] = [(None, "the write never ran", {}, "")]

        def worker() -> None:
            """
            Background write; completes the reactor completion when done.
            """
            try:
                thread_name = threading.current_thread().name
                chelper.get_ffi()[1].set_thread_name(thread_name.encode("utf-8"))
            except Exception:
                pass
            try:
                result[0] = _write_blocking(target, spool_id, overrides,
                                            excluded)
            except Exception as e:
                result[0] = (None, str(e), {}, "")
            reactor.register_async_callback(lambda et: completion.complete(None))

        # Name the worker after its reader, clipped to the 15-char OS limit.
        tname = ("afc_wr_" + target.name.split(":")[-1])[:15]
        threading.Thread(target=worker, daemon=True, name=tname).start()
        completion.wait(reactor.monotonic() + WRITE_TIMEOUT_S)
        return result[0]

    return _with_staging(target, lane, _body)


def _read_keys(unit: Any) -> Dict[str, Any]:
    """
    The brand keys a read needs, taken off the owning unit.

    Every host-side unit resolves these into the same attribute names at ready
    (via resolve_rfid_keys).

    :param unit: the owning unit
    :return dict: keyword args for read_tag
    """
    return {
        "bambu_master_key": getattr(unit, "bambu_master_key", None),
        "creality_key": getattr(unit, "creality_key", None),
        "creality_encryption_key": getattr(unit, "creality_encryption_key",
                                            None),
    }


def _read_blocking(target: RfidWriteTarget) -> Tuple[Optional[dict],
                                                     Optional[str]]:
    """
    Read whatever tag is in a reader's field, through its own link.

    Uses the same open_link/prepare/release the write does. A reader without a
    register link (the U1's, driven by OpenRFID) cannot be read this way.

    :param target: the reader to read
    :return tuple: (raw read_tag dict, error message)
    """
    link = target.open_link()
    if link is None:
        return None, f"{target.name} is offline"
    if not hasattr(link, "reg_read"):
        return None, (f"{target.name} cannot be read on demand here; its "
                      f"reader scans on its own")
    target.prepare(link)
    try:
        tag = read_tag(link, **_read_keys(target.unit))
    finally:
        target.release(link)
    if tag is None or tag.get("uid") is None:
        return None, "no tag in the reader's field"
    return tag, None


#: The tag record spans pages 4-39 (Anycubic layout + AFC block, 144 bytes).
#: Erasing zeroes exactly that, leaving the reserved pages either side alone.
_ERASE_LEN = 144


def _erase_blocking(target: RfidWriteTarget) -> Tuple[Optional[str],
                                                      Optional[str]]:
    """
    Zero a tag's record so it reads blank, on a worker or reactor per target.

    Uses the register link or the U1's payload hand-off. Spoolman is not
    touched, so the spool's card_uids keeps the UID.

    :param target: the reader holding the tag
    :return tuple: (uid, error)
    """
    blank = b"\x00" * _ERASE_LEN
    handoff = getattr(target, "write_payload", None)
    if handoff is not None:
        return handoff(blank)
    link = target.open_link()
    if link is None:
        return None, f"{target.name} is offline"
    target.prepare(link)
    try:
        return write_tag(link, blank)
    finally:
        target.release(link)


def run_erase(target: RfidWriteTarget) -> Tuple[Optional[str], Optional[str]]:
    """
    Run an erase where the transport allows: worker thread or reactor.

    :param target: the reader holding the tag
    :return tuple: (uid, error)
    """
    if not target.threaded:
        return _erase_blocking(target)
    reactor = target.unit.reactor
    completion = reactor.completion()
    result: List[Any] = [(None, "the erase never ran")]

    def worker() -> None:
        """
        Background erase; completes the reactor completion when done.
        """
        try:
            thread_name = threading.current_thread().name
            chelper.get_ffi()[1].set_thread_name(thread_name.encode("utf-8"))
        except Exception:
            pass
        try:
            result[0] = _erase_blocking(target)
        except Exception as e:
            result[0] = (None, str(e))
        reactor.register_async_callback(lambda et: completion.complete(None))

    threading.Thread(target=worker, daemon=True,
                     name="afc_rfid_er").start()
    completion.wait(reactor.monotonic() + WRITE_TIMEOUT_S)
    return result[0]


def _classic_write_run(target: RfidWriteTarget, block: int, data16: bytes,
                       key6: bytes) -> Tuple[Optional[str], Optional[str]]:
    """
    Write one Classic block through a reader's link, on the right thread.

    :param target: the reader holding the tag
    :param block: absolute block number
    :param data16: exactly 16 bytes
    :param key6: 6-byte sector key
    :return Tuple[Optional[str], Optional[str]]: (uid, error)
    """
    def _do() -> Tuple[Optional[str], Optional[str]]:
        """
        Open the link and write the block.

        :return Tuple[Optional[str], Optional[str]]: (uid, error)
        """
        link = target.open_link()
        if link is None:
            return None, f"{target.name} is offline"
        if not hasattr(link, "reg_read"):
            return None, (f"{target.name} cannot do a Classic write here; its "
                          f"reader is driven elsewhere")
        target.prepare(link)
        try:
            return classic_write_block(link, block, data16, key6)
        finally:
            target.release(link)

    if not target.threaded:
        return _do()
    reactor = target.unit.reactor
    completion = reactor.completion()
    result: List[Any] = [(None, "the write never ran")]

    def worker() -> None:
        """
        Background Classic write; completes the reactor completion when done.
        """
        try:
            thread_name = threading.current_thread().name
            chelper.get_ffi()[1].set_thread_name(thread_name.encode("utf-8"))
        except Exception:
            pass
        try:
            result[0] = _do()
        except Exception as e:
            result[0] = (None, str(e))
        reactor.register_async_callback(lambda et: completion.complete(None))
    threading.Thread(target=worker, daemon=True,
                     name="afc_cw").start()
    completion.wait(reactor.monotonic() + WRITE_TIMEOUT_S)
    return result[0]


def cmd_AFC_RFID_CLASSIC_WRITE(printer: Any, gcmd: Any) -> None:
    """
    Write one 16-byte block of a MIFARE Classic tag.

    The low-level block writer for authoring Bambu-format data on a blank or
    magic Classic card (default key FFFFFFFFFFFF). Refuses sector trailers and
    block 0, and reads back to confirm. Genuine Bambu tags are write-locked by
    their access bits and will refuse this.

    Usage
    -------
    `AFC_RFID_CLASSIC_WRITE READER=<name> BLOCK=<n> DATA=<32 hex chars> [KEY=<12 hex chars>]`

    Example
    -------
    ```
    AFC_RFID_CLASSIC_WRITE READER=reader0 BLOCK=4 DATA=00112233445566778899AABBCCDDEEFF
    ```
    """
    target, err = resolve_reader(printer, gcmd.get("READER", None))
    if target is None:
        error_str = f"AFC_RFID_CLASSIC_WRITE: {err}"
        raise gcmd.error(error_str)
    block = gcmd.get_int("BLOCK", minval=1, maxval=62)
    data_hex = gcmd.get("DATA", "").strip()
    try:
        data16 = bytes.fromhex(data_hex)
    except ValueError:
        error_str = "AFC_RFID_CLASSIC_WRITE: DATA is not hex"
        raise gcmd.error(error_str)
    if len(data16) != 16:
        error_str = (f"AFC_RFID_CLASSIC_WRITE: DATA is {len(data16)} bytes, "
                     f"need 16 (32 hex chars)")
        raise gcmd.error(error_str)
    key_hex = gcmd.get("KEY", "").strip()
    if key_hex:
        try:
            key6 = bytes.fromhex(key_hex)
        except ValueError:
            error_str = "AFC_RFID_CLASSIC_WRITE: KEY is not hex"
            raise gcmd.error(error_str)
        if len(key6) != 6:
            error_str = "AFC_RFID_CLASSIC_WRITE: KEY must be 12 hex chars"
            raise gcmd.error(error_str)
    else:
        key6 = CLASSIC_DEFAULT_KEY
    uid, err = _classic_write_run(target, block, data16, key6)
    if err:
        where = f" (tag {uid})" if uid else ""
        error_str = f"AFC_RFID_CLASSIC_WRITE: {err}{where}"
        raise gcmd.error(error_str)
    gcmd.respond_info(
        f"AFC_RFID_CLASSIC_WRITE: wrote block {block} on tag {uid} "
        f"({target.name}), verified.")


def cmd_AFC_RFID_ERASE(printer: Any, gcmd: Any) -> None:
    """
    Erase the tag at a reader: zero its record so it reads blank again.

    For re-purposing a sticker. The factory UID and Spoolman are unchanged;
    this only clears the on-tag data.

    Usage
    -------
    `AFC_RFID_ERASE READER=<name>`

    Example
    -------
    ```
    AFC_RFID_ERASE READER=reader0
    ```
    """
    target, err = resolve_reader(printer, gcmd.get("READER", None))
    if target is None:
        error_str = f"AFC_RFID_ERASE: {err}"
        raise gcmd.error(error_str)
    uid, err = run_erase(target)
    if err:
        where = f" (tag {uid})" if uid else ""
        error_str = f"AFC_RFID_ERASE: {err}{where}"
        raise gcmd.error(error_str)
    gcmd.respond_info(
        f"AFC_RFID_ERASE: erased tag {uid} on {target.name}. It now reads "
        f"blank; write or enroll it again to re-use it.")


def run_read(target: RfidWriteTarget) -> Tuple[Optional[dict], Optional[str]]:
    """
    Run a read where the transport allows (worker thread or reactor), mirroring
    run_write.

    :param target: the reader to read
    :return tuple: (raw read_tag dict, error message)
    """
    if not target.threaded:
        return _read_blocking(target)
    reactor = target.unit.reactor
    completion = reactor.completion()
    result: List[Any] = [(None, "the read never ran")]

    def worker() -> None:
        """
        Background read; completes the reactor completion when done.
        """
        try:
            thread_name = threading.current_thread().name
            chelper.get_ffi()[1].set_thread_name(thread_name.encode("utf-8"))
        except Exception:
            pass
        try:
            result[0] = _read_blocking(target)
        except Exception as e:
            result[0] = (None, str(e))
        reactor.register_async_callback(lambda et: completion.complete(None))

    threading.Thread(target=worker, daemon=True,
                     name="afc_rfid_rd").start()
    completion.wait(reactor.monotonic() + WRITE_TIMEOUT_S)
    return result[0]


def _fmt_tag(tag: dict) -> str:
    """
    One-line-per-field readout of a read_tag result for the console.

    :param tag: the raw read_tag dict
    :return str: formatted lines
    """
    lines = [f"uid: {tag.get('uid')}", f"type: {tag.get('tag_type')}"]
    # NTAG21x with a capability container only: names the chip and its memory.
    if tag.get("chip"):
        lines.append(f"chip: {tag['chip']}")
    if tag.get("user_bytes"):
        lines.append(f"user memory: {tag['user_bytes']} bytes")
    fil = tag.get("filament") or {}
    if not fil:
        lines.append("filament: (undecoded: blank or unknown layout)")
        return "\n".join(lines)
    argb = fil.get("color_argb")
    order = [("manufacturer", "brand"), ("type", "material"),
             ("detailed", "detail"), ("sku", "sku"), ("weight_g", "weight g"),
             ("diameter_mm", "diameter"), ("density", "density"),
             ("hotend_min_c", "hotend min"), ("hotend_max_c", "hotend max"),
             ("bed_temp_c", "bed"), ("spool_id", "spool id")]
    for key, label in order:
        if fil.get(key) not in (None, "", 0):
            lines.append(f"{label}: {fil[key]}")
    if argb:
        lines.append(f"colour: #{argb & 0xFFFFFF:06X}")
    return "\n".join(lines)


def cmd_AFC_RFID_READ(printer: Any, gcmd: Any) -> None:
    """
    Read the tag at a reader and report it. The read-side twin of
    AFC_RFID_WRITE.

    Reports the UID and, when the layout is recognised, the decoded filament.
    Nothing is applied to a lane; it is the quick way to confirm a write. Run
    AFC_RFID_READERS for names.

    Usage
    -------
    `AFC_RFID_READ READER=<name>`

    Example
    -------
    ```
    AFC_RFID_READ READER=reader0
    ```
    """
    target, err = resolve_reader(printer, gcmd.get("READER", None))
    if target is None:
        error_str = f"AFC_RFID_READ: {err}"
        raise gcmd.error(error_str)
    tag, err = run_read(target)
    if err:
        error_str = f"AFC_RFID_READ: {err}"
        raise gcmd.error(error_str)
    gcmd.respond_info(f"AFC_RFID_READ on {target.name}\n{_fmt_tag(tag)}")


def cmd_AFC_RFID_READERS(printer: Any, gcmd: Any) -> None:
    """
    List every reader that can write a tag, and which are online.

    Usage
    -------
    `AFC_RFID_READERS`

    Example
    -------
    ```
    AFC_RFID_READERS
    ```
    """
    reg = _registry(printer)
    if not reg:
        gcmd.respond_info(
            "AFC_RFID_READERS: none registered. A host-side reader "
            "(BoxTurtle, ACE2, ViViD or OpenAMS) is what provides them; the "
            "Bambu AMS and the U1 read their own tags and cannot write.")
        return
    lines = ["AFC RFID readers (READER= takes any name below)"]
    for target in reg.values():
        state = "online" if _online(target) else "OFFLINE"
        lines.append(f"  {target.name}: {target.label} [{state}]")
    gcmd.respond_info("\n".join(lines))


def _overrides_from(gcmd: Any, command: str) -> Dict[str, Any]:
    """
    Collect the explicit tag fields a command was given.

    Only parameters actually present are returned, so they can be layered over
    a Spoolman record without blanking what it supplied.

    :param gcmd: The Klipper GCodeCommand
    :param command: the calling command's name, for its error messages
    :return dict: field name -> value
    :raises gcmd.error: on a malformed colour
    """
    out: Dict[str, Any] = {}
    for name, key in (("BRAND", "manufacturer"), ("SKU", "sku"),
                      ("TYPE", "ftype")):
        if gcmd.get(name, None) is not None:
            out[key] = gcmd.get(name)
    if gcmd.get("COLOR", None) is not None:
        color = gcmd.get("COLOR").lstrip("#")
        try:
            out["color_argb"] = 0xFF000000 | (int(color, 16) & 0xFFFFFF)
        except ValueError:
            error_str = f"{command}: COLOR={color} is not hex RRGGBB"
            raise gcmd.error(error_str)
    for name, key in (("WEIGHT", "weight_g"), ("HOTEND_MIN", "hotend_min_c"),
                      ("HOTEND_MAX", "hotend_max_c"), ("BED", "bed_temp_c"),
                      ("DRY_TEMP", "drying_temp_c"),
                      ("DRY_TIME", "drying_time_h")):
        if gcmd.get(name, None) is not None:
            out[key] = gcmd.get_int(name, 0, minval=0)
    for name, key in (("DIAMETER", "diameter_mm"), ("DENSITY", "density")):
        if gcmd.get(name, None) is not None:
            out[key] = gcmd.get_float(name, 0., above=0.)
    return out


def tag_from_fields(uid: Optional[str], fields: Dict[str, Any]) -> dict:
    """
    Build what a read of a freshly written tag returns, from what was written.

    Uses the same decoders read_tag does on the same bytes, so the lane gets
    exactly what its next scan would give without a second read.

    :param uid: the tag's UID, hex
    :param fields: the fields the write encoded
    :return dict: shaped like a read_tag result
    """
    image = b"\x00" * ANYCUBIC_IMAGE_START + encode_tag_payload(**fields)
    fil = dict(decode_anycubic(image) or {})
    fil.update(decode_afc_block(image) or {})
    return {"uid": uid, "sak": 0, "tag_type": "MifareUltralight",
            "filament": fil or None}


def apply_written(target: RfidWriteTarget, lane_name: str,
                  uid: Optional[str], fields: Dict[str, Any]) -> str:
    """
    Give the lane the tag just written to its spool. Runs on the reactor.

    The lane takes the new tag the way a scan would, through the unit's own
    apply (Spoolman match included). A unit that tracks tags per lane can
    define apply_written_tag(lane_name, tag) to note the UID as well.

    :param target: the reader the tag was written on
    :param lane_name: the lane whose spool carries the tag
    :param uid: the tag's UID
    :param fields: the fields the write encoded
    :return str: a note for the command's reply, empty when applied quietly
    """
    unit = target.unit
    tag = tag_from_fields(uid, fields)
    try:
        hook = getattr(unit, "apply_written_tag", None)
        if hook is not None:
            hook(lane_name, tag)
            return f"applied to {lane_name}"
        afc = getattr(unit, "afc", None)
        lane = (getattr(afc, "lanes", None) or {}).get(lane_name)
        apply = getattr(unit, "apply_to_lane", None)
        if lane is None or apply is None:
            return f"not applied: {lane_name} is not a lane this unit can set"
        apply(lane, tag)
        send = getattr(lane, "send_lane_data", None)
        if send is not None:
            send()
        save = getattr(afc, "save_vars", None)
        if save is not None:
            save()
        return f"applied to {lane_name}"
    except Exception as e:
        return f"but applying it to {lane_name} failed: {e}"


def cmd_AFC_RFID_WRITE(printer: Any, gcmd: Any) -> None:
    """
    Write a blank NTAG sticker so a third-party spool reads like a branded one.

    Hold the sticker against the named reader's antenna and run it, or pass
    LANE= so the unit pulls the spool's tag to its reader first and the lane
    takes the written tag afterwards; LANE= alone also picks the reader. With
    SPOOL= the fields come from that Spoolman record and the tag's UID is bound
    to it; any explicit parameter overrides Spoolman. Run AFC_RFID_READERS for
    the names.

    Usage
    -------
    `AFC_RFID_WRITE READER=<name> [LANE=<lane>] [SPOOL=<id>] [TYPE=PLA] [BRAND=] [SKU=]
    [COLOR=RRGGBB] [WEIGHT=] [DIAMETER=] [DENSITY=] [HOTEND_MIN=] [HOTEND_MAX=] [BED=]
    [DRY_TEMP=] [DRY_TIME=]`

    Example
    -------
    ```
    AFC_RFID_WRITE READER=reader0 TYPE=PETG BRAND=Generic COLOR=FF0000 WEIGHT=1000
    ```
    """
    target, err = resolve_reader(printer, gcmd.get("READER", None),
                                 gcmd.get("LANE", None))
    if target is None:
        error_str = f"AFC_RFID_WRITE: {err}"
        raise gcmd.error(error_str)
    spool_id = gcmd.get_int("SPOOL", 0, minval=0)
    overrides = _overrides_from(gcmd, "AFC_RFID_WRITE")
    if not spool_id and not overrides:
        error_str = ("AFC_RFID_WRITE: nothing to write. Give a SPOOL= id to "
                     "take the record from Spoolman, or at least TYPE=.")
        raise gcmd.error(error_str)
    if not spool_id:
        overrides.setdefault("ftype", "PLA")
        overrides.setdefault("diameter_mm", 1.75)
    lane = gcmd.get("LANE", None)
    try:
        uid, err, fields, note = run_write(target, spool_id, overrides,
                                           lane=lane)
    except StageError as e:
        error_str = f"AFC_RFID_WRITE: {e}"
        raise gcmd.error(error_str)
    if err:
        where = f" (tag {uid})" if uid else ""
        error_str = f"AFC_RFID_WRITE: {err}{where}"
        raise gcmd.error(error_str)
    if lane:
        applied = apply_written(target, lane, uid, fields)
        note = f"{note}, {applied}" if note else applied
    weight = fields.get("weight_g")
    gcmd.respond_info(
        f"AFC_RFID_WRITE: wrote tag {uid} on {target.name}: "
        f"{fields.get('manufacturer') or 'unbranded'} "
        f"{fields.get('ftype') or 'filament'}"
        f"{f', {weight}g' if weight else ''}"
        f"{f' ({note})' if note else ''}. "
        f"Stick it on the spool and scan to confirm.")


def _create_spool_blocking(unit: Any,
                           fields: Dict[str, Any]) -> Tuple[int, Optional[str]]:
    """
    Create a Spoolman vendor + filament + spool from tag fields. Worker only.

    For a blank tag whose spool is not in Spoolman yet.

    :param unit: the owning unit, for its Spoolman client
    :param fields: the tag fields (manufacturer, ftype, color_argb, ...)
    :return tuple: (new spool id, error message)
    """
    client = _cached_spoolman_client(getattr(unit, "afc", None))
    if client is None:
        return 0, "no Spoolman client"
    brand = fields.get("manufacturer") or "Generic"
    ftype = fields.get("ftype") or "PLA"
    argb = fields.get("color_argb")
    color_hex = f"{argb & 0xFFFFFF:06X}" if argb else None
    weight = fields.get("weight_g")
    sku = fields.get("sku") or None
    try:
        vendor = client.get_or_create_vendor(brand)
        vid = vendor.get("id") if isinstance(vendor, dict) else None
        fil = client.create_filament(
            name=sku or f"{brand} {ftype}", vendor_id=vid, material=ftype,
            density=fields.get("density"), diameter=fields.get("diameter_mm"),
            color_hex=color_hex,
            settings_extruder_temp=fields.get("hotend_max_c") or None,
            settings_bed_temp=fields.get("bed_temp_c") or None,
            weight=weight, article_number=sku)
        if not isinstance(fil, dict) or "id" not in fil:
            return 0, f"Spoolman filament create failed ({fil})"
        spool = client.create_spool(fil["id"], initial_weight=weight)
        if not isinstance(spool, dict) or "id" not in spool:
            return 0, f"Spoolman spool create failed ({spool})"
        return int(spool["id"]), None
    except Exception as e:
        return 0, f"Spoolman create failed: {e}"


def run_enroll(target: RfidWriteTarget, spool_id: int,
               overrides: Dict[str, Any],
               lane: Optional[str] = None) -> Tuple[Any, ...]:
    """
    Enroll a blank tag: create the Spoolman spool if one was not named, then
    write the tag and bind its UID, on a worker where the transport allows it.

    :param target: the reader holding the blank tag
    :param spool_id: an existing Spoolman spool to link to, 0 to create one
    :param overrides: the tag fields typed in
    :param lane: the lane whose spool holds the tag, for a reader that must spin
        it into range; None when the tag is already at the coil
    :return tuple: (uid, error, fields, note)
    """
    def _do(excluded: Optional[Callable[[str], bool]]) -> Tuple[Any, ...]:
        """
        Create the spool if needed, then write and bind the tag.

        :param excluded: uid_hex -> bool test for tags to pass over
        :return tuple: (uid, error, fields, note)
        """
        sid, prefix = spool_id, ""
        if not sid:
            # Confirm a tag is present before creating a Spoolman record, so a
            # missing tag cannot leave an orphan spool. A hand-off reader (the
            # U1) relies on the write's own no-tag check instead.
            _tag, rerr = _read_blocking(target)
            if rerr and ("no tag" in rerr or "offline" in rerr):
                return None, rerr, {}, ""
            sid, err = _create_spool_blocking(target.unit, overrides)
            if err:
                return None, err, {}, ""
            prefix = f"created spool {sid}; "
            # The new record is authoritative; only fields Spoolman cannot hold
            # (HOTEND_MIN, DRY_*) are taken from the typed params.
            tag_only = {key: overrides[key]
                        for key in ("hotend_min_c", "drying_temp_c", "drying_time_h")
                        if key in overrides}
            uid, err, fields, note = _write_blocking(target, sid, tag_only,
                                                     excluded)
            if err:
                # Name the new spool so it is reused rather than created again.
                err = (f"{err}. Spool {sid} was created in Spoolman: give "
                       f"SPOOL={sid} when you enroll a writable tag, or delete it")
        else:
            uid, err, fields, note = _write_blocking(target, sid, overrides,
                                                     excluded)
        return uid, err, fields, prefix + (note or "")

    def _dispatch(excluded: Optional[Callable[[str], bool]]
                  ) -> Tuple[Any, ...]:
        """
        Run the enroll, on a worker thread when the target allows it.

        :param excluded: uid_hex -> bool test for tags to pass over
        :return tuple: (uid, error, fields, note)
        """
        if not target.threaded:
            return _do(excluded)
        reactor = target.unit.reactor
        completion = reactor.completion()
        result: List[Any] = [(None, "the enroll never ran", {}, "")]

        def worker() -> None:
            """
            Background enroll; completes the reactor completion when done.
            """
            try:
                thread_name = threading.current_thread().name
                chelper.get_ffi()[1].set_thread_name(thread_name.encode("utf-8"))
            except Exception:
                pass
            try:
                result[0] = _do(excluded)
            except Exception as e:
                result[0] = (None, str(e), {}, "")
            reactor.register_async_callback(
                lambda et: completion.complete(None))

        threading.Thread(target=worker, daemon=True,
                         name="afc_rfid_en").start()
        completion.wait(reactor.monotonic() + WRITE_TIMEOUT_S + 30.0)
        return result[0]

    return _with_staging(target, lane, _dispatch)


def cmd_AFC_RFID_ENROLL(printer: Any, gcmd: Any) -> None:
    """
    Enroll a blank tag into Spoolman: create (or link) a spool, program the
    tag, and bind its UID, in one step for a fresh third-party spool.

    With SPOOL= it links to that existing spool; without it, a new spool is
    created from the fields typed in. Either way the tag then resolves straight
    to the spool on the next scan. LANE= lets a spool reader spin the tag into
    the coil's field before writing; the blank-tag prompt fills it in.

    Usage
    -------
    `AFC_RFID_ENROLL READER=<name> [LANE=<lane>] [SPOOL=<id>] [TYPE=PLA] [BRAND=] [SKU=]
    [COLOR=RRGGBB] [WEIGHT=] [DIAMETER=] [DENSITY=] [HOTEND_MIN=] [HOTEND_MAX=] [BED=]
    [DRY_TEMP=] [DRY_TIME=]`

    Example
    -------
    ```
    AFC_RFID_ENROLL LANE=lane1 TYPE=PLA BRAND=Generic COLOR=00FF00 WEIGHT=1000
    ```
    """
    target, err = resolve_reader(printer, gcmd.get("READER", None),
                                 gcmd.get("LANE", None))
    if target is None:
        error_str = f"AFC_RFID_ENROLL: {err}"
        raise gcmd.error(error_str)
    spool_id = gcmd.get_int("SPOOL", 0, minval=0)
    overrides = _overrides_from(gcmd, "AFC_RFID_ENROLL")
    if not spool_id:
        overrides.setdefault("ftype", "PLA")
        overrides.setdefault("diameter_mm", 1.75)
    lane = gcmd.get("LANE", None)
    try:
        uid, err, fields, note = run_enroll(target, spool_id, overrides,
                                            lane=lane)
    except StageError as e:
        error_str = f"AFC_RFID_ENROLL: {e}"
        raise gcmd.error(error_str)
    if err:
        where = f" (tag {uid})" if uid else ""
        error_str = f"AFC_RFID_ENROLL: {err}{where}"
        raise gcmd.error(error_str)
    if lane:
        applied = apply_written(target, lane, uid, fields)
        note = f"{note}, {applied}" if note else applied
    weight = fields.get("weight_g")
    gcmd.respond_info(
        f"AFC_RFID_ENROLL: tag {uid} on {target.name}: "
        f"{fields.get('manufacturer') or 'unbranded'} "
        f"{fields.get('ftype') or 'filament'}"
        f"{f', {weight}g' if weight else ''}"
        f"{f' ({note})' if note else ''}.")
