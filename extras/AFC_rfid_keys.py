# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Shared RFID decode keys, configured once and used by every host-side AFC RFID
# reader (ACE 2 Pro, ViViD/BTT MMS). A reader's own section can override any key.
#
#   [AFC_rfid_keys]
#   bambu_master_key: <32 hex chars>        # enables Bambu tag decode
#   creality_key: <32 hex chars>            # enables Creality CFS decode (both
#   creality_encryption_key: <32 hex chars> # keys required together)
#
# The Snapmaker U1 decodes tags via the OpenRFID daemon, so its keys live in
# OpenRFID's openrfid_user.cfg instead.
from __future__ import annotations
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from configfile import ConfigWrapper


def _hex_key(config: "ConfigWrapper", name: str) -> Optional[bytes]:
    """
    Parse a hex key config value to bytes, or None when unset. Raises a config
    error on a non-hex value so a typo is caught at startup, not on the first read.

    :param config: Klipper config wrapper for the section.
    :param name: Config option name to read.
    :return Optional[bytes]: Parsed key bytes, or None when unset.
    """
    raw = (config.get(name, "") or "").strip()
    if not raw:
        return None
    try:
        return bytes.fromhex(raw)
    except ValueError:
        error_str = f"AFC_rfid_keys: '{name}' must be hex characters, got {raw!r}"
        raise config.error(error_str)


class AFC_rfid_keys:
    """Holds the shared RFID decode keys parsed from ``[AFC_rfid_keys]``."""

    def __init__(self, config: "ConfigWrapper") -> None:
        """
        Store the shared RFID decode keys parsed from the config section.

        :param config: Klipper config wrapper for the section.
        """
        self.bambu_master_key = _hex_key(config, "bambu_master_key")
        self.creality_key = _hex_key(config, "creality_key")
        self.creality_encryption_key = _hex_key(config, "creality_encryption_key")


def load_config(config: "ConfigWrapper") -> "AFC_rfid_keys":
    """
    Klipper entry point for the ``[AFC_rfid_keys]`` section, the one shared
    place every host-side RFID reader looks up its decode keys.

    :param config: Klipper config wrapper for the section.
    :return AFC_rfid_keys: The parsed shared-keys object.
    """
    return AFC_rfid_keys(config)
