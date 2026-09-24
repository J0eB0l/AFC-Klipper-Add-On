"""Unit tests for extras/AFC_rfid_keys.py."""

from __future__ import annotations

import configparser
from typing import Dict, Optional

import pytest

from extras.AFC_rfid_keys import _hex_key, AFC_rfid_keys, load_config
from tests.conftest import MockConfig


class RfidKeysConfigError(configparser.Error):
    """The section's ``config.error``, kept distinct so a test can see an error came through it."""


class RfidKeysSectionConfig(MockConfig):
    """
    The ``[AFC_rfid_keys]`` section. Klipper's ConfigWrapper.error is the
    exception class, so ``config.error(msg)`` only builds the error and the
    caller has to raise it; MockConfig.error would raise on its own.
    """

    error = RfidKeysConfigError

    def __init__(self, values: Optional[Dict[str, Optional[str]]] = None) -> None:
        """
        :param values: option values as written in the section; unset when absent
        """
        super().__init__(name="AFC_rfid_keys", values=values)


class TestHexKey:
    def test_hex_key_parses_to_bytes(self):
        config = RfidKeysSectionConfig({"k": "aabbcc"})
        assert _hex_key(config, "k") == b"\xaa\xbb\xcc"

    def test_hex_key_unset_is_none(self):
        # Also pins that a default is passed: MockConfig raises on an unset option without one.
        assert _hex_key(RfidKeysSectionConfig(), "missing") is None
        # Whitespace alone strips to unset; bytes.fromhex would read it as b"".
        assert _hex_key(RfidKeysSectionConfig({"k": "   "}), "k") is None

    def test_hex_key_none_value_is_none(self):
        # A wrapper handing back None for the option reads as unset, not a crash.
        assert _hex_key(RfidKeysSectionConfig({"k": None}), "k") is None

    def test_hex_key_bad_hex_raises_config_error(self):
        config = RfidKeysSectionConfig({"bambu_master_key": "nothex!"})
        with pytest.raises(configparser.Error) as raised:
            _hex_key(config, "bambu_master_key")
        assert type(raised.value) is RfidKeysConfigError
        assert str(raised.value) == (
            "AFC_rfid_keys: 'bambu_master_key' must be hex characters, got 'nothex!'")


class TestAFCrfidkeysInit:
    def test_keys_parsed_from_section(self):
        keys = AFC_rfid_keys(RfidKeysSectionConfig({
            "bambu_master_key": "00112233445566778899aabbccddeeff",
            "creality_key": "0f0e0d",
            # creality_encryption_key left unset
        }))
        assert vars(keys) == {
            "bambu_master_key": (b"\x00\x11\x22\x33\x44\x55\x66\x77"
                                 b"\x88\x99\xaa\xbb\xcc\xdd\xee\xff"),
            "creality_key": b"\x0f\x0e\x0d",
            "creality_encryption_key": None,
        }

    def test_each_key_comes_from_its_own_option(self):
        keys = AFC_rfid_keys(RfidKeysSectionConfig({
            "bambu_master_key": "01",
            "creality_key": "02",
            "creality_encryption_key": "03",
        }))
        assert vars(keys) == {
            "bambu_master_key": b"\x01",
            "creality_key": b"\x02",
            "creality_encryption_key": b"\x03",
        }


class TestLoadConfig:
    def test_load_config_returns_keys_object(self):
        keys = load_config(RfidKeysSectionConfig({"bambu_master_key": "abcd"}))
        assert type(keys) is AFC_rfid_keys
        assert vars(keys) == {
            "bambu_master_key": b"\xab\xcd",
            "creality_key": None,
            "creality_encryption_key": None,
        }
