"""
Builders for the AFC_ACE2_rfid tests: AFC_ACE2_RFID bound to an afcACE2, and
its reader links (_Ace2RegLink, Ace2Link).

Kept apart from ace_helpers so that file stays the ACE units' own.
"""

from __future__ import annotations

from typing import Any, Callable, List, Mapping, Optional

from extras.AFC_ACE2_rfid import AFC_ACE2_RFID, Ace2Link, _Ace2RegLink
from tests.ace_helpers import AceConfig, AcePrinter, DEFAULT, make_ace2_unit


def make_ace2_rfid(values: Optional[Mapping[str, Any]] = None, *, ace2: Any = DEFAULT,
                   printer: Optional[AcePrinter] = None, bind: bool = True,
                   clear_records: bool = True,
                   cls: Optional[type] = None) -> AFC_ACE2_RFID:
    """
    Build AFC_ACE2_RFID through its real __init__ as [AFC_ACE2_rfid].

    bind runs its klippy:ready handler (_on_ready) once and drops it: that
    resolves AFC, finds the afcACE2 by its "AFC_ACE2 <name>" section and
    registers the tag writers.

    :param values: [AFC_ACE2_rfid] options
    :param ace2: the afcACE2 to bind; DEFAULT builds one, None binds nothing
    :param printer: shared printer, defaults to ace2's or a new one
    :param bind: run _on_ready
    :param clear_records: clear every record once wiring is done
    :param cls: AFC_ACE2_RFID class to build, e.g. a test's module copy
    :return AFC_ACE2_RFID: the reader object
    """
    if ace2 is not DEFAULT and ace2 is not None and printer is None:
        printer = ace2.printer
    printer = printer if printer is not None else AcePrinter()
    if ace2 is DEFAULT:
        ace2 = make_ace2_unit(printer=printer, clear_records=False)
    if ace2 is not None and ace2.printer is not printer:
        error_str = "ace2 must be built on the same printer"
        raise ValueError(error_str)
    config = AceConfig("AFC_ACE2_rfid", printer, values)
    obj = (cls or AFC_ACE2_RFID)(config)
    printer.add_section("AFC_ACE2_rfid", obj, values)
    if bind:
        obj._on_ready()
        printer.remove_event_handler("klippy:ready", obj._on_ready)
    if clear_records:
        printer.clear_records()
    return obj


def make_ace2_reg_link(slot: int = 0, *, ace2: Any = DEFAULT, cls: Optional[type] = None,
                       **options: Any) -> _Ace2RegLink:
    """
    Build an _Ace2RegLink over an afcACE2, so its commands land in the unit's
    connection (ace2._ace.commands).

    :param slot: reader chip-select index
    :param ace2: the afcACE2; DEFAULT builds one
    :param cls: link class to build, e.g. a module copy's _Ace2RegLink
    :param options: _Ace2RegLink keywords (power_index, reg_timeout, ...)
    :return _Ace2RegLink: the link
    """
    if ace2 is DEFAULT:
        ace2 = make_ace2_unit()
    return (cls or _Ace2RegLink)(ace2, slot, **options)


class FakeTransport:
    """Ace2Link transport: records each frame, answers with .replies in order
    (last repeats), b"" when none is set."""

    def __init__(self, *replies: bytes) -> None:
        """
        :param replies: response frames
        """
        self.frames: List[bytes] = []
        self.replies: List[bytes] = list(replies)

    def __call__(self, frame: bytes) -> bytes:
        """
        :param frame: request frame
        :return bytes: the next reply
        """
        self.frames.append(bytes(frame))
        if not self.replies:
            return b""
        return self.replies.pop(0) if len(self.replies) > 1 else self.replies[0]


def make_ace2_link(slot: int = 0, *, transport: Optional[Callable[[bytes], bytes]] = None,
                   ftype: int = 0x0000, cls: Optional[type] = None) -> Ace2Link:
    """
    Build an Ace2Link over a recording transport.

    :param slot: reader slot
    :param transport: frame transport, defaults to a FakeTransport
    :param ftype: frame type word
    :param cls: link class to build, e.g. a module copy's Ace2Link
    :return Ace2Link: the link; its transport is link._tx
    """
    return (cls or Ace2Link)(transport if transport is not None else FakeTransport(),
                             slot=slot, ftype=ftype)

