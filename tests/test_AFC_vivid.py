"""
Unit tests for extras/AFC_vivid.py

Covers:
  - AFC_vivid: class constants
  - _get_lane_selector_state: returns correct bool from fila_selector
  - _get_selector_enabled: returns stepper enabled status
  - calibration_lane_message: returns informative string
  - cmd_AFC_SELECT_LANE: dispatches to select_lane or gcmd.error
"""

from __future__ import annotations

import configparser
from unittest.mock import MagicMock, patch, PropertyMock
import pytest

from extras.AFC_vivid import AFC_vivid
from extras.AFC_BoxTurtle import afcBoxTurtle
from extras.AFC_lane import AFCLane
from tests.test_AFC_lane import _make_afc_lane
from typing import Any, Dict, Optional, Tuple
from unittest import mock

from extras.AFC_lane import AFCLaneState, AFCMoveWarning, AssistActive, SpeedMode
from extras.AFC_vivid import load_config_prefix
from tests.conftest import MockAFC, MockConfig, MockPrinter


def _build_vivid_config(name: str = "ViViD_1",
                        values: Optional[Dict[str, Any]] = None
                        ) -> Tuple[MockConfig, MockPrinter, MockAFC]:
    """
    Config for one [AFC_vivid <name>] section, with its drive and selector
    [AFC_stepper] sections present and a stepper object registered for each.

    :param name: Unit name, the last word of the section name
    :param values: Options to set or override in the section
    :return tuple: The config, its printer and the printer's AFC object
    """
    afc = MockAFC()
    printer = MockPrinter(afc=afc)
    options: Dict[str, Any] = {"drive_stepper": "drive", "selector_stepper": "selector"}
    options.update(values or {})
    config = MockConfig(name=f"AFC_vivid {name}", printer=printer, values=options)
    for stepper in (options["drive_stepper"], options["selector_stepper"]):
        config.fileconfig.add_section(f"AFC_stepper {stepper}")
        printer.objects[f"AFC_stepper {stepper}"] = MagicMock()
    return config, printer, afc


def _build_vivid_unit(values: Optional[Dict[str, Any]] = None) -> AFC_vivid:
    """
    An AFC_vivid named ViViD_1 through its real __init__, on a printer with no
    stepper_enable object, so its selector reads as disabled.

    :param values: Options to set or override in the unit's section
    :return AFC_vivid: The unit
    """
    config, _printer, _afc = _build_vivid_config(values=values)
    return AFC_vivid(config)


def _build_vivid_lane() -> MagicMock:
    """
    ViViD lane1 as AFC_vivid reads it: prepped, calibrated, short of its load
    sensor, untooled, with its selector switch clear and a hub to retract by.

    :return MagicMock: The lane
    """
    lane = MagicMock(spec=AFCLane)
    lane.name = "lane1"
    lane.afc = MagicMock()
    lane.selector_endstop_name = "lane1_selector"
    lane.load_endstop_name = "lane1_load"
    lane.load_es = "lane1_load"
    lane.fila_selector = MagicMock()
    lane.fila_selector.get_status.return_value = {"filament_detected": False}
    lane.selector_cal_dis = None
    lane.short_moves_speed = 50.0
    lane.short_moves_accel = 400.0
    lane.prep_state = True
    lane.raw_load_state = False
    lane.calibrated_lane = True
    lane.dist_hub = 200.0
    lane.loaded_to_hub = False
    lane.tool_loaded = False
    lane.hub_obj = MagicMock(hub_clear_move_dis=65.0)
    lane.remember_spool = False
    lane.status = AFCLaneState.LOADED
    lane.td1_data = {}
    lane.current_led_state = "loaded"
    return lane



# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_vivid_config(name="ViViD_1", values=None, drive_stepper="drive",
                       selector_stepper="selector", add_stepper_sections=True):
    """Build the MockConfig/MockPrinter/MockAFC trio needed to construct an
    AFC_vivid via its real __init__ (chained through afcBoxTurtle.__init__
    and afcUnit.__init__), without constructing it -- lets a test drive
    AFC_vivid(config) itself.

    Registers [AFC_stepper <drive_stepper>]/[AFC_stepper <selector_stepper>]
    config sections and matching MagicMock stepper objects in the printer's
    object cache so afcUnit._lookup_objects resolves drive_stepper_obj/
    selector_stepper_obj successfully.
    """
    from tests.conftest import MockAFC, MockConfig, MockPrinter

    afc = MockAFC()
    printer = MockPrinter(afc=afc)

    all_values = {
        "drive_stepper": drive_stepper,
        "selector_stepper": selector_stepper,
    }
    if values:
        all_values.update(values)

    config = MockConfig(name=f"AFC_vivid {name}", printer=printer, values=all_values)

    if add_stepper_sections:
        config.fileconfig.add_section(f"AFC_stepper {drive_stepper}")
        config.fileconfig.add_section(f"AFC_stepper {selector_stepper}")
        printer._objects[f"AFC_stepper {drive_stepper}"] = MagicMock()
        printer._objects[f"AFC_stepper {selector_stepper}"] = MagicMock()

    return config, printer, afc


def _make_vivid(name="ViViD_1", values=None, **kwargs):
    """Build an AFC_vivid instance via its real __init__ (chained through
    afcBoxTurtle.__init__ and afcUnit.__init__)."""
    config, printer, afc = _make_vivid_config(name=name, values=values, **kwargs)
    return AFC_vivid(config)


def _make_lane(name="lane1", has_selector=True):
    lane = MagicMock()
    lane.name = name
    lane.selector_endstop = "selector_pin" if has_selector else None
    lane.selector_endstop_name = "lane1_selector"
    lane.load_endstop_name = "lane1_load"
    lane.prep_endstop_name = "lane1_prep"
    lane.dist_hub = 200
    lane.calibrated_lane = True
    if has_selector:
        lane.fila_selector = MagicMock()
        lane.fila_selector.get_status.return_value = {"filament_detected": False}
    else:
        del lane.fila_selector  # make attribute not exist
    return lane


def _make_real_spool():
    """Build a real AFCSpool via its actual __init__, for tests that need
    clear_values() to run for real rather than as a mocked call."""
    from extras.AFC_spool import AFCSpool
    from tests.conftest import MockConfig, MockPrinter

    config = MockConfig(printer=MockPrinter())
    return AFCSpool(config)


# ── Class constants ───────────────────────────────────────────────────────────

class TestVivdConstants:
    def test_valid_cam_angles(self):
        assert AFC_vivid.VALID_CAM_ANGLES == [30, 45, 60]

    def test_calibration_distance(self):
        assert AFC_vivid.CALIBRATION_DISTANCE == 5000

    def test_lane_overshoot(self):
        assert AFC_vivid.LANE_OVERSHOOT == 200

    def test_is_subclass_of_box_turtle(self):
        assert issubclass(AFC_vivid, afcBoxTurtle)


# ── __init__ ──────────────────────────────────────────────────────────────────

class TestVividInit:
    """Covers AFC_vivid's own __init__ logic: reading its config variables
    and registering AFC_UNSELECT_LANE. afcUnit/afcBoxTurtle's own __init__
    bodies are pre-existing behavior out of scope here."""

    def test_defaults_when_not_set_in_config(self):
        unit = _make_vivid()
        assert unit.type == "ViViD"
        assert unit.current_selected_lane is None
        assert unit.home_state is False
        assert unit.prep_homed is False
        assert unit.failed_to_home is False
        assert unit.selector_homing_speed == 150
        assert unit.selector_homing_accel == 150
        assert unit.max_selector_movement == 800
        assert unit._eject_to_calibrate is True

    def test_reads_overridden_values_from_config(self):
        unit = _make_vivid(
            drive_stepper="drive_a",
            selector_stepper="selector_b",
            values={
                "type": "CustomViViD",
                "selector_homing_speed": 200.0,
                "selector_homing_accel": 175.0,
                "max_selector_movement": 900.0,
                "enable_sensors_in_gui": True,
            },
        )
        assert unit.type == "CustomViViD"
        assert unit.drive_stepper == "drive_a"
        assert unit.selector_stepper == "selector_b"
        assert unit.selector_homing_speed == 200.0
        assert unit.selector_homing_accel == 175.0
        assert unit.max_selector_movement == 900.0
        assert unit.enable_sensors_in_gui is True

    def test_enable_sensors_in_gui_falls_back_to_afc_value(self):
        """When not set in this unit's own config, enable_sensors_in_gui
        falls back to afc.enable_sensors_in_gui rather than a fixed
        default."""
        config, printer, afc = _make_vivid_config()
        afc.enable_sensors_in_gui = True
        unit = AFC_vivid(config)
        assert unit.enable_sensors_in_gui is True

    def test_registers_afc_unselect_lane_mux_command(self):
        config, printer, afc = _make_vivid_config()
        afc.function.register_mux_command = MagicMock()
        unit = AFC_vivid(config)
        afc.function.register_mux_command.assert_called_once_with(
            afc.show_macros, 'AFC_UNSELECT_LANE', 'UNIT', unit.name,
            unit.cmd_AFC_UNSELECT_LANE, unit.cmd_AFC_UNSELECT_LANE_help,
            unit.cmd_AFC_UNSELECT_LANE_options,
        )

    def test_lookup_objects_populated_and_command_registered(self):
        config, printer, afc = _build_vivid_config(
            values={"drive_stepper": "vivid_drive", "selector_stepper": "vivid_selector"})
        afc.show_macros = False

        unit = AFC_vivid(config)

        assert unit.drive_stepper_obj is printer.lookup_object("AFC_stepper vivid_drive")
        assert unit.selector_stepper_obj is printer.lookup_object("AFC_stepper vivid_selector")
        assert afc.function.register_mux_command.call_args_list == [
            mock.call(False, "AFC_UNSELECT_LANE", "UNIT", "ViViD_1", unit.cmd_AFC_UNSELECT_LANE,
                      unit.cmd_AFC_UNSELECT_LANE_help, unit.cmd_AFC_UNSELECT_LANE_options)]
        assert afc.logger.messages == []


# ── module import guards ────────────────────────────────────────────────────

class TestModuleImportGuards:
    """Covers the module-level try/except import guards at the top of
    AFC_vivid.py, each of which raises a config_error built from a
    formatted message when the corresponding import fails."""

    def _reload_with_failing_import(self, monkeypatch, blocked_module):
        """Loads a throwaway copy of AFC_vivid.py under a separate module
        name with one import made to fail, without mutating the real
        extras.AFC_vivid module object shared by the rest of the suite."""
        import builtins
        import importlib.util
        import sys as _sys

        real_import = builtins.__import__

        def fake_import(name, *args, **kwargs):
            if name == blocked_module:
                raise ImportError(f"blocked {blocked_module} for test")
            return real_import(name, *args, **kwargs)

        monkeypatch.setattr(builtins, "__import__", fake_import)

        spec = importlib.util.spec_from_file_location(
            "extras.AFC_vivid_import_guard_test", "extras/AFC_vivid.py")
        module = importlib.util.module_from_spec(spec)
        _sys.modules["extras.AFC_vivid_import_guard_test"] = module
        try:
            spec.loader.exec_module(module)
        finally:
            del _sys.modules["extras.AFC_vivid_import_guard_test"]

    def test_error_str_import_failure_raises_config_error(self, monkeypatch):
        with pytest.raises(configparser.Error, match="ERROR_STR"):
            self._reload_with_failing_import(monkeypatch, "extras.AFC_utils")

    def test_box_turtle_import_failure_raises_config_error(self, monkeypatch):
        with pytest.raises(configparser.Error, match="AFC_BoxTurtle"):
            self._reload_with_failing_import(monkeypatch, "extras.AFC_BoxTurtle")

    def test_afc_lane_import_failure_raises_config_error(self, monkeypatch):
        with pytest.raises(configparser.Error, match="AFC_lane"):
            self._reload_with_failing_import(monkeypatch, "extras.AFC_lane")


# ── load_config_prefix ───────────────────────────────────────────────────────

class TestLoadConfigPrefix:
    def test_returns_afc_vivid_instance(self):
        from extras.AFC_vivid import load_config_prefix
        config, printer, afc = _make_vivid_config()
        result = load_config_prefix(config)
        assert isinstance(result, AFC_vivid)

    def test_returns_afc_vivid_instance_built_from_config(self):
        config, printer, afc = _build_vivid_config(name="my_vivid")

        instance = load_config_prefix(config)

        assert type(instance) is AFC_vivid
        assert instance.name == "my_vivid"
        assert instance.type == "ViViD"
        assert instance.printer is printer
        assert afc.logger.messages == []


# ── _move_lane ──────────────────────────────────────────────────
class Test_MoveLane:
    def test_returns_prep_true_filament_loaded(self):
        unit = _make_vivid()
        lane = _make_lane(has_selector=True)
        lane.prep_state = True
        lane.spool_id = 10
        lane.remember_spool = True
        lane.tool_loaded = False
        lane.move_to.return_value = (True, 100.0, False)
        result = unit._move_lane(lane, 1, True)
        assert result is True
        assert lane.spool_id == 10
    
    def test_homed_but_already_tool_loaded_skips_hub_retract(self):
        """When homing succeeds but the lane is already tool_loaded, the
        clearance retract off the hub must not run."""
        unit = _make_vivid()
        lane = _make_lane(has_selector=True)
        lane.prep_state = True
        lane.tool_loaded = True
        lane.move_to.return_value = (True, 100.0, False)
        unit._move_lane(lane, 1, True)
        lane.move_to.assert_called_once()  # only the load-sensor move, no retract

    def test_homed_but_no_hub_obj_skips_hub_retract(self):
        """When homing succeeds and tool_loaded is False (so the hub retract
        would otherwise run), a None hub_obj must still skip it rather than
        raising -- independent coverage of the `and lane.hub_obj` operand,
        separate from the tool_loaded operand covered above."""
        unit = _make_vivid()
        lane = _make_lane(has_selector=True)
        lane.prep_state = True
        lane.tool_loaded = False
        lane.hub_obj = None
        lane.move_to.return_value = (True, 100.0, False)
        unit._move_lane(lane, 1, True)
        lane.move_to.assert_called_once()  # only the load-sensor move, no retract

    def test_returns_prep_true_filament_not_loaded(self):
        unit = _make_vivid()
        lane = _make_lane(has_selector=True)
        lane.prep_state = True
        lane.loaded_to_hub = True
        lane.spool_id = 10
        lane.remember_spool = True
        lane.tool_loaded = False
        lane.move_to.return_value = (False, 100.0, False)
        result = unit._move_lane(lane, 1, True)
        assert result is False
        assert lane.tool_loaded is False
        assert lane.loaded_to_hub is False
        assert lane.spool_id == 10
    
    def test_returns_prep_false_filament_not_loaded(self):
        unit = _make_vivid()
        lane = _make_lane(has_selector=True)
        lane.afc.spool = _make_real_spool()
        lane.prep_state = False
        lane.loaded_to_hub = True
        lane.spool_id = 10
        lane.remember_spool = True
        lane.tool_loaded = False
        lane.move_to.return_value = (False, 100.0, False)
        result = unit._move_lane(lane, 1, True)
        assert result is False
        assert lane.tool_loaded is False
        assert lane.loaded_to_hub is False
        assert lane.spool_id == 10
    
    def test_returns_prep_false_filament_not_loaded_not_remember_spool(self):
        unit = _make_vivid()
        lane = _make_lane(has_selector=True)
        lane.afc.spool = _make_real_spool()
        lane.prep_state = False
        lane.loaded_to_hub = True
        lane.spool_id = 10
        lane.remember_spool = False
        lane.tool_loaded = False
        lane.move_to.return_value = (False, 100.0, False)
        result = unit._move_lane(lane, 1, True)
        assert result is False
        assert lane.tool_loaded is False
        assert lane.loaded_to_hub is False
        assert lane.spool_id is None

    LOAD_MOVE = mock.call(200.0, SpeedMode.SHORT, endstop="lane1_load",
                          assist_active=AssistActive.DYNAMIC, use_homing=True)

    def test_tool_loaded_skips_retract_and_reset(self):
        unit = _build_vivid_unit()
        lane = _build_vivid_lane()
        lane.tool_loaded = True
        lane.status = AFCLaneState.TOOLED
        lane.current_led_state = "tool_loaded"
        lane.td1_data = {"color": "FF0000"}
        lane.move_to.return_value = (True, 190.0, AFCMoveWarning.NONE)

        result = unit._move_lane(lane, 1, True)

        assert result is True
        assert lane.loaded_to_hub is True
        # Only the homing move: the hub is there, so tool_loaded alone stops the retract.
        assert lane.move_to.call_args_list == [self.LOAD_MOVE]
        # The reset block would clear all of these and drop the spool.
        assert lane.tool_loaded is True
        assert lane.status == AFCLaneState.TOOLED
        assert lane.td1_data == {"color": "FF0000"}
        assert lane.current_led_state == "tool_loaded"
        assert lane.afc.spool.clear_values.call_args_list == []
        assert unit.logger.messages == []

    def test_untooled_lane_retracts_off_the_load_sensor(self):
        unit = _build_vivid_unit()
        lane = _build_vivid_lane()
        lane.move_to.return_value = (True, 190.0, AFCMoveWarning.NONE)

        result = unit._move_lane(lane, 1, True)

        assert result is True
        assert lane.loaded_to_hub is True
        assert lane.move_to.call_args_list == [
            self.LOAD_MOVE, mock.call(-65.0, SpeedMode.SHORT, use_homing=False)]
        assert lane.current_led_state == "loaded"
        assert unit.logger.messages == []

# ── _get_lane_selector_state ──────────────────────────────────────────────────

class TestGetLaneSelectorState:
    def test_returns_filament_detected_when_selector_present(self):
        unit = _make_vivid()
        lane = _make_lane(has_selector=True)
        lane.fila_selector.get_status.return_value = {"filament_detected": True}
        result = unit._get_lane_selector_state(lane)
        assert result is True

    def test_returns_false_when_selector_not_detected(self):
        unit = _make_vivid()
        lane = _make_lane(has_selector=True)
        lane.fila_selector.get_status.return_value = {"filament_detected": False}
        result = unit._get_lane_selector_state(lane)
        assert result is False

    def test_returns_false_when_no_selector_attribute(self):
        unit = _make_vivid()
        lane = MagicMock(spec=[])  # No attributes
        result = unit._get_lane_selector_state(lane)
        assert result is False


# ── _get_selector_enabled ─────────────────────────────────────────────────────

class TestGetSelectorEnabled:
    def test_returns_true_when_stepper_enabled(self):
        unit = _make_vivid()
        stepper_enable = MagicMock()
        stepper_enable.get_status.return_value = {
            "steppers": {f"AFC_stepper {unit.selector_stepper}": True}
        }
        unit.printer._objects["stepper_enable"] = stepper_enable
        result = unit._get_selector_enabled()
        assert result is True

    def test_returns_false_when_stepper_not_found(self):
        unit = _make_vivid()
        # No stepper_enable in printer objects → returns False
        unit.printer._objects = {}
        result = unit._get_selector_enabled()
        assert result is False


# ── calibration_lane_message ──────────────────────────────────────────────────

class TestCalibrationLaneMessage:
    def test_returns_non_empty_string(self):
        unit = _make_vivid()
        msg = unit.calibration_lane_message()
        assert isinstance(msg, str)
        assert len(msg) > 0

    def test_message_mentions_reinsert(self):
        unit = _make_vivid()
        msg = unit.calibration_lane_message()
        assert "reinsert" in msg.lower() or "insert" in msg.lower()

    def test_message_mentions_vivid(self):
        unit = _make_vivid()
        msg = unit.calibration_lane_message()
        assert "vivid" in msg.lower() or "ViViD" in msg


# ── cmd_AFC_SELECT_LANE ────────────────────────────────────────────────────────

class TestCmdAfcSelectLane:
    def test_calls_select_lane_when_lane_found(self):
        unit = _make_vivid()
        lane = _make_lane("lane1")
        unit.afc.lanes = {"lane1": lane}
        unit.select_lane = MagicMock(return_value=(True, 15.0))
        from tests.conftest import MockGCodeCommand
        gcmd = MockGCodeCommand(params={"LANE": "lane1"})
        unit.cmd_AFC_SELECT_LANE(gcmd)
        unit.select_lane.assert_called_once_with(lane)

    def test_logs_success_when_homed(self):
        unit = _make_vivid()
        lane = _make_lane("lane1")
        unit.afc.lanes = {"lane1": lane}
        unit.select_lane = MagicMock(return_value=(True, 15.0))
        from tests.conftest import MockGCodeCommand
        gcmd = MockGCodeCommand(params={"LANE": "lane1"})
        unit.cmd_AFC_SELECT_LANE(gcmd)
        info_msgs = [m for lvl, m in unit.logger.messages if lvl == "info"]
        assert any("lane1" in m for m in info_msgs)

    def test_logs_error_when_homing_fails(self):
        unit = _make_vivid()
        lane = _make_lane("lane1")
        unit.afc.lanes = {"lane1": lane}
        unit.select_lane = MagicMock(return_value=(False, 0))
        from tests.conftest import MockGCodeCommand
        gcmd = MockGCodeCommand(params={"LANE": "lane1"})
        unit.cmd_AFC_SELECT_LANE(gcmd)
        error_msgs = [m for lvl, m in unit.logger.messages if lvl == "error"]
        assert any("lane1" in m for m in error_msgs)

    def test_calls_gcmd_error_when_lane_not_found(self):
        unit = _make_vivid()
        unit.afc.lanes = {}
        from tests.conftest import MockGCodeCommand
        gcmd = MockGCodeCommand(params={"LANE": "missing_lane"})
        unit.cmd_AFC_SELECT_LANE(gcmd)
        gcmd.error.assert_called()


# ── handle_connect ────────────────────────────────────────────────────────────

class TestVividHandleConnect:
    def test_handle_connect_sets_logo(self):
        unit = _make_vivid()
        unit.set_logo_color = MagicMock()
        unit.handle_connect()
        assert "ViViD Ready" in unit.logo

    def test_handle_connect_sets_logo_error(self):
        unit = _make_vivid()
        unit.set_logo_color = MagicMock()
        unit.handle_connect()
        assert "ViViD Not Ready" in unit.logo_error

    def test_handle_connect_registers_unit_in_afc(self):
        unit = _make_vivid(name="vivid_1")
        unit.set_logo_color = MagicMock()
        unit.handle_connect()
        assert unit.afc.units.get("vivid_1") is unit


# ── system_Test ───────────────────────────────────────────────────────────────

class TestVividSystemTest:
    def test_system_test_calls_super_with_movement_disabled(self):
        unit = _make_vivid()
        lane = MagicMock()
        with MagicMock() as super_mock:
            from extras.AFC_BoxTurtle import afcBoxTurtle
            with MagicMock() as patch_target:
                from unittest.mock import patch
                with patch.object(afcBoxTurtle, 'system_Test', return_value="ok") as mock_st:
                    result = unit.system_Test(lane, 0.5, True, True)
                    mock_st.assert_called_once_with(lane, 0.5, True, enable_movement=False)


# ── prep_post_load ────────────────────────────────────────────────────────────

class TestPrepPostLoad:
    def test_returns_none(self):
        unit = _make_vivid()
        lane = MagicMock()
        result = unit.prep_post_load(lane)
        assert result is None


# ── unselect_lane ─────────────────────────────────────────────────────────────

class TestUnselectLane:
    def test_calls_selector_stepper_move(self):
        unit = _make_vivid()
        unit.unselect_lane()
        unit.selector_stepper_obj.move.assert_called_once_with(50, 150, 150, False)


# ── move_to_hub ───────────────────────────────────────────────────────────────

class TestMoveToHub:
    def test_delegates_to_lane_move_to(self):
        from extras.AFC_lane import SpeedMode, MoveDirection, AssistActive, AFCMoveWarning
        unit = _make_vivid()
        lane = MagicMock()
        lane.move_to.return_value = (True, 100.0, AFCMoveWarning.NONE)
        lane.load_es = "load_endstop"
        result = unit.move_to_hub(lane, 100.0, MoveDirection.POS)
        lane.move_to.assert_called_once()
        assert result == (True, 100.0, AFCMoveWarning.NONE)

    def test_returns_homed_distance_warn_tuple(self):
        from extras.AFC_lane import SpeedMode, MoveDirection, AFCMoveWarning
        unit = _make_vivid()
        lane = MagicMock()
        lane.move_to.return_value = (False, 50.0, AFCMoveWarning.WARN)
        lane.load_es = "load_endstop"
        homed, dist, warn = unit.move_to_hub(lane, 50.0, MoveDirection.NEG)
        assert homed is False
        assert warn is AFCMoveWarning.WARN


# ── select_lane ───────────────────────────────────────────────────────────────

class TestSelectLane:
    def test_returns_false_and_zero_when_no_selector_endstop(self):
        """Matches the declared tuple[bool, float|int] return type instead of
        falling off the end and implicitly returning None."""
        unit = _make_vivid()
        lane = _make_lane("lane1", has_selector=False)
        lane.selector_endstop_name = None
        result = unit.select_lane(lane)
        assert result == (False, 0)

    def test_returns_true_and_zero_when_already_selected_and_enabled(self):
        unit = _make_vivid()
        lane = _make_lane("lane1", has_selector=True)
        lane.fila_selector.get_status.return_value = {"filament_detected": True}
        # stepper enabled
        stepper_enable = MagicMock()
        stepper_enable.get_status.return_value = {
            "steppers": {f"AFC_stepper {unit.selector_stepper}": True}
        }
        unit.printer._objects["stepper_enable"] = stepper_enable
        result = unit.select_lane(lane)
        assert result == (True, 0.0)

    def test_calls_homing_when_not_selected(self):
        unit = _make_vivid()
        unit._selector_cal_dis_adjust = MagicMock()
        lane = _make_lane("lane1", has_selector=True)
        lane.fila_selector.get_status.return_value = {"filament_detected": False}
        unit.printer._objects = {}  # no stepper_enable → enabled=False
        unit.selector_stepper_obj.do_homing_move.return_value = (True, 15.0)
        homed, dist = unit.select_lane(lane)
        assert homed is True
        assert dist == 15.0
        unit.selector_stepper_obj.do_homing_move.assert_called_once()

    def test_calls_selector_cal_dis_adjust_after_homing(self):
        unit = _make_vivid()
        unit._selector_cal_dis_adjust = MagicMock()
        lane = _make_lane("lane1", has_selector=True)
        lane.fila_selector.get_status.return_value = {"filament_detected": False}
        unit.printer._objects = {}
        unit.selector_stepper_obj.do_homing_move.return_value = (True, 15.0)
        unit.select_lane(lane)
        unit._selector_cal_dis_adjust.assert_called_once_with(lane)

    def test_calls_unselect_lane_when_disabled_but_selector_triggered(self):
        """When stepper not enabled but selector triggered, unselect_lane is called first."""
        unit = _make_vivid()
        unit._selector_cal_dis_adjust = MagicMock()
        lane = _make_lane("lane1", has_selector=True)
        lane.fila_selector.get_status.return_value = {"filament_detected": True}
        # stepper disabled (no stepper_enable object)
        unit.printer._objects = {}
        unit.unselect_lane = MagicMock()
        unit.selector_stepper_obj.do_homing_move.return_value = (True, 10.0)
        unit.select_lane(lane)
        unit.unselect_lane.assert_called_once()

    HOMING_MOVE = mock.call(movepos=750.0, speed=120.0, accel=240.0,
                            endstop_spec="lane1_selector", assist_active=False)
    HOMING_LOGS = [("debug", "ViViD: Selecting lane1"),
                   ("debug", "ViViD: Homing done, success:True, distance:15.257")]

    @staticmethod
    def _select_with_cal_dis(cal_dis: Optional[float]
                             ) -> Tuple[AFC_vivid, Tuple[bool, float]]:
        """
        Select lane1 on a unit whose selector is disabled and clear, so the cam
        homes, with the lane's selector_cal_dis set to cal_dis.

        :param cal_dis: The lane's selector_cal_dis
        :return tuple: The unit and what select_lane returned
        """
        unit = _build_vivid_unit(values={"selector_homing_speed": 120.0,
                                         "selector_homing_accel": 240.0,
                                         "max_selector_movement": 750.0})
        lane = _build_vivid_lane()
        lane.selector_cal_dis = cal_dis
        unit.selector_stepper_obj.do_homing_move.return_value = (True, 15.257)
        return unit, unit.select_lane(lane)

    def test_calibration_move_runs_when_cal_dis_nonzero(self):
        unit, result = self._select_with_cal_dis(5.0)

        assert result == (True, 15.26)
        assert unit.selector_stepper_obj.do_homing_move.call_args_list == [self.HOMING_MOVE]
        assert unit.selector_stepper_obj.move.call_args_list == [mock.call(5.0, 50.0, 400.0, False)]
        assert unit.logger.messages == self.HOMING_LOGS

    def test_calibration_move_skipped_when_cal_dis_none(self):
        unit, result = self._select_with_cal_dis(None)

        assert result == (True, 15.26)
        assert unit.selector_stepper_obj.do_homing_move.call_args_list == [self.HOMING_MOVE]
        assert unit.selector_stepper_obj.move.call_args_list == []
        assert unit.logger.messages == self.HOMING_LOGS

    def test_calibration_move_skipped_when_cal_dis_zero(self):
        unit, result = self._select_with_cal_dis(0.0)

        assert result == (True, 15.26)
        assert unit.selector_stepper_obj.do_homing_move.call_args_list == [self.HOMING_MOVE]
        assert unit.selector_stepper_obj.move.call_args_list == []
        assert unit.logger.messages == self.HOMING_LOGS


# ── calibrate_lane ────────────────────────────────────────────────────────────

class TestCalibrateLane:
    def test_returns_correct_tuple(self):
        from extras.AFC_lane import AFCLaneState
        unit = _make_vivid()
        lane = MagicMock()
        unit.eject_lane = MagicMock()
        result = unit.calibrate_lane(lane, 0)
        assert result == (True, "calibration_lane", 0)

    def test_sets_loaded_to_hub_false(self):
        unit = _make_vivid()
        lane = MagicMock()
        unit.eject_lane = MagicMock()
        unit.calibrate_lane(lane, 0)
        assert lane.loaded_to_hub is False

    def test_sets_calibrated_lane_false(self):
        unit = _make_vivid()
        lane = MagicMock()
        unit.eject_lane = MagicMock()
        unit.calibrate_lane(lane, 0)
        assert lane.calibrated_lane is False

    def test_calls_eject_lane(self):
        unit = _make_vivid()
        lane = MagicMock()
        unit.eject_lane = MagicMock()
        unit.calibrate_lane(lane, 5.0)
        unit.eject_lane.assert_called_once_with(lane)

    def test_calls_lane_unloaded(self):
        unit = _make_vivid()
        lane = MagicMock()
        unit.eject_lane = MagicMock()
        unit.calibrate_lane(lane, 0)
        lane.unit_obj.lane_unloaded.assert_called_once_with(lane)


# ── _get_selector_enabled except branch ───────────────────────────────────────

class TestGetSelectorEnabledExceptBranch:
    def test_returns_false_when_stepper_key_missing_from_steppers(self):
        """Covers the except branch when the specific stepper key is absent."""
        unit = _make_vivid()
        stepper_enable = MagicMock()
        stepper_enable.get_status.return_value = {"steppers": {}}  # key not present
        unit.printer._objects["stepper_enable"] = stepper_enable
        result = unit._get_selector_enabled()
        assert result is False


# ── prep_load ─────────────────────────────────────────────────────────────────

class TestPrepLoad:
    def test_calibrated_lane_sets_loaded_to_hub(self):
        unit = _make_vivid()
        lane = _make_afc_lane()
        lane.calibrated_lane = True
        lane.dist_hub = 200.0
        lane.move_to = MagicMock(return_value = (True, 200.0, False))
        lane.prep_state = True
        lane.loaded_to_hub = False
        lane.hub_obj = MagicMock()
        unit.lane_loading = MagicMock()
        unit.select_lane = MagicMock()
        unit.lane_loaded = MagicMock()
        # reads: loop check (False) -> feed -> loop check (True, exit) -> return.
        with patch.object(type(lane), "raw_load_state", new_callable=PropertyMock) as mock_prop:
            mock_prop.side_effect = [False, True, True]
            unit.prep_load(lane)

        assert lane.loaded_to_hub is True
        unit.lane_loading.assert_called_once_with(lane)
        unit.select_lane.assert_called_once_with(lane, sel_prep=True)
        unit.lane_loaded.assert_called_once_with(lane)

    def test_disables_steppers_and_selects_loaded_lane(self):
        unit = _make_vivid()
        lane = MagicMock()
        lane.calibrated_lane = True
        lane.dist_hub = 200.0
        lane.move_to.return_value = (True, 200.0, False)
        unit.lane_loading = MagicMock()
        unit.select_lane = MagicMock()
        unit.lane_loaded = MagicMock()

        unit.prep_load(lane)

        unit.selector_stepper_obj.do_enable.assert_called_with(False)
        unit.drive_stepper_obj.do_enable.assert_called_with(False)
        unit.afc.function.select_loaded_lane.assert_called_once()

    def test_uncalibrated_lane_updates_dist_hub_and_config(self):
        unit = _make_vivid()
        lane = _make_afc_lane()
        lane.calibrated_lane = False
        lane.prep_state = True
        lane.hub_obj = MagicMock()
        lane.move_to = MagicMock(return_value = (True, 300.0, False))
        unit.lane_loading = MagicMock()
        unit.select_lane = MagicMock()
        unit.lane_loaded = MagicMock()
        with patch.object(type(lane), "raw_load_state", new_callable=PropertyMock) as mock_prop:
            mock_prop.side_effect = [False, True, True]

            unit.prep_load(lane)

        assert lane.calibrated_lane is True
        assert lane.dist_hub == round(300.0, 2) + AFC_vivid.LANE_OVERSHOOT
        unit.afc.function.ConfigRewrite.assert_called()
    
    def test_uncalibrated_lane_updates_dist_hub_and_config_two_tries(self):
        # An RFID stop-on-detect splits the feed: the homing call returns homed but
        # the real sensor is still false, so the feed re-issues. dist_hub must be
        # the SUM of the segment distances (origin->sensor), not just the last one.
        unit = _make_vivid()
        lane = _make_afc_lane()
        lane.calibrated_lane = False
        lane.prep_state = True
        lane.hub_obj = MagicMock()
        lane.move_to = MagicMock(return_value = (True, 300.0, False))
        unit.lane_loading = MagicMock()
        unit.select_lane = MagicMock()
        unit.lane_loaded = MagicMock()
        # two segments (sensor still false), then the real sensor trips.
        with patch.object(type(lane), "raw_load_state", new_callable=PropertyMock) as mock_prop:
            mock_prop.side_effect = [False, False, True, True]

            unit.prep_load(lane)

        assert lane.calibrated_lane is True
        assert lane.dist_hub == round(600.0, 2) + AFC_vivid.LANE_OVERSHOOT
        unit.afc.function.ConfigRewrite.assert_called()
    
    def test_uncalibrated_lane_updates_dist_hub_and_config_failed(self):
        unit = _make_vivid()
        lane = _make_afc_lane()
        lane.calibrated_lane = False
        lane.prep_state = True
        lane.move_to = MagicMock(return_value = (False, 300.0, False))
        lane.dist_hub = 0.0
        unit.lane_loading = MagicMock()
        unit.select_lane = MagicMock()
        unit.lane_loaded = MagicMock()
        # genuine misses (homed False) -> retried twice, then give up; final read.
        with patch.object(type(lane), "raw_load_state", new_callable=PropertyMock) as mock_prop:
            mock_prop.side_effect = [False, False, False, False]

            unit.prep_load(lane)

        assert lane.calibrated_lane is False
        assert lane.dist_hub == 0.0
        unit.afc.function.ConfigRewrite.assert_not_called()
        # Failure should be reported/logged
        error_msgs = [m for lvl, m in unit.logger.messages if lvl == "error"]
        assert error_msgs
    
    def test_uncalibrated_lane_updates_dist_hub_no_prep(self):
        unit = _make_vivid()
        lane = MagicMock()
        lane.calibrated_lane = False
        lane.prep_state = False
        lane.raw_load_state = False                    # real bool, not a mock obj
        lane.move_to.return_value = (True, 300.0, False)
        unit.lane_loading = MagicMock()
        unit.select_lane = MagicMock()
        unit.lane_loaded = MagicMock()

        unit.prep_load(lane)

        assert lane.calibrated_lane is False

    def test_homed_with_no_hub_obj_skips_retract_move(self):
        """When lane.hub_obj is None, the retract-past-load-sensor move must
        be skipped rather than raising, but lane_loaded still runs."""
        unit = _make_vivid()
        lane = _make_afc_lane()
        lane.calibrated_lane = True
        lane.dist_hub = 200.0
        lane.hub_obj = None
        move_to_mock = MagicMock(return_value=(True, 200.0, False))
        lane.move_to = move_to_mock
        lane.prep_state = True
        lane.loaded_to_hub = False
        unit.lane_loading = MagicMock()
        unit.select_lane = MagicMock()
        unit.lane_loaded = MagicMock()
        with patch.object(type(lane), "raw_load_state", new_callable=PropertyMock) as mock_prop:
            mock_prop.side_effect = [False, True, True]
            unit.prep_load(lane)

        # Only the one move_to call from the homing loop, no extra retract call
        assert move_to_mock.call_count == 1
        unit.lane_loaded.assert_called_once_with(lane)

    def test_not_homed_skips_lane_loaded(self):
        unit = _make_vivid()
        lane = MagicMock()
        lane.calibrated_lane = True
        lane.dist_hub = 200.0
        lane.prep_state = True
        lane.raw_load_state = False                    # real bool, never trips
        lane.move_to.return_value = (False, 0.0, False)
        unit.lane_loading = MagicMock()
        unit.select_lane = MagicMock()
        unit.lane_loaded = MagicMock()

        unit.prep_load(lane)

        unit.lane_loaded.assert_not_called()
        # Steppers are disabled regardless of homing result
        unit.selector_stepper_obj.do_enable.assert_called_with(False)


class TestStageAndLoad:
    def test_no_rfid_listener_is_a_plain_homing_feed(self):
        unit = _make_vivid()
        lane = MagicMock()
        lane.calibrated_lane = True
        lane.dist_hub = 200.0
        lane.prep_state = True
        lane.raw_load_state = False

        def move_to(dist, speed, assist_active=None, endstop=None,
                    use_homing=False):
            lane.raw_load_state = True                  # the sensor trips (homed)
            return (True, 200.0, False)

        lane.move_to.side_effect = move_to
        # MockPrinter.send_event does nothing -> no RFID fake-abort.
        homed, dist = unit._stage_and_load(lane)
        assert homed is True and dist == 200.0
        assert lane.move_to.call_count == 1             # one homing feed, no chunks
        for call in lane.move_to.call_args_list:
            assert call.kwargs.get("use_homing") is True

    def test_brackets_the_load_with_stage_read_events(self):
        # begin/end events bracket the normal homing load, every move is a homing
        # move (no chunking), and the real sensor decides completion.
        unit = _make_vivid()
        lane = MagicMock()
        lane.calibrated_lane = True
        lane.dist_hub = 500.0
        lane.prep_state = True
        lane.raw_load_state = False
        lane.load_endstop_name = "load"
        events = []
        move_at_begin = {}

        def move_to(dist, speed, assist_active=None, endstop=None,
                    use_homing=False):
            assert use_homing is True                  # only homing moves
            lane.raw_load_state = True
            return (True, dist, False)

        lane.move_to.side_effect = move_to

        def send_event(name, l=None):
            events.append(name)
            if name.endswith("stage_read_begin"):
                move_at_begin["calls"] = lane.move_to.call_count

        unit.printer.send_event = send_event
        homed, _dist = unit._stage_and_load(lane)
        assert "afc_vivid:stage_read_begin" in events
        assert "afc_vivid:stage_read_end" in events
        # begin fires BEFORE the feed, end AFTER.
        assert move_at_begin["calls"] == 0
        begin = events.index("afc_vivid:stage_read_begin")
        assert begin < events.index("afc_vivid:stage_read_end")
        assert homed is True

    CALIBRATED_FEED = mock.call(200.0, SpeedMode.LONG, assist_active=AssistActive.NO,
                                endstop="lane1_load", use_homing=True)

    @staticmethod
    def _feed_trips_sensor(lane: MagicMock, moved: float) -> None:
        """
        Make each feed of lane move it moved mm and trip its load sensor.

        :param lane: The lane being staged
        :param moved: Distance each feed reports
        """
        def feed(*args: Any, **kwargs: Any) -> Tuple[bool, float, AFCMoveWarning]:
            lane.raw_load_state = True
            return True, moved, AFCMoveWarning.NONE
        lane.move_to.side_effect = feed

    def test_stage_read_begin_error_is_logged(self):
        unit = _build_vivid_unit()
        lane = _build_vivid_lane()
        self._feed_trips_sensor(lane, 180.0)
        ended: list = []

        def failing_begin(staged: MagicMock) -> None:
            raise RuntimeError("boom")

        unit.printer.register_event_handler("afc_vivid:stage_read_begin", failing_begin)
        unit.printer.register_event_handler("afc_vivid:stage_read_end", ended.append)

        result = unit._stage_and_load(lane)

        assert result == (True, 180.0)
        assert lane.move_to.call_args_list == [self.CALIBRATED_FEED]
        assert ended == [lane]
        assert unit.logger.messages == [("error", "ViViD stage read begin error: boom")]

    def test_stage_read_end_error_is_logged(self):
        unit = _build_vivid_unit()
        lane = _build_vivid_lane()
        self._feed_trips_sensor(lane, 180.0)
        begun: list = []

        def failing_end(staged: MagicMock) -> None:
            raise RuntimeError("bang")

        unit.printer.register_event_handler("afc_vivid:stage_read_begin", begun.append)
        unit.printer.register_event_handler("afc_vivid:stage_read_end", failing_end)

        result = unit._stage_and_load(lane)

        assert result == (True, 180.0)
        assert lane.move_to.call_args_list == [self.CALIBRATED_FEED]
        assert begun == [lane]
        assert unit.logger.messages == [("error", "ViViD stage read end error: bang")]

    def test_an_empty_prep_feeds_nothing(self):
        unit = _build_vivid_unit()
        lane = _build_vivid_lane()
        lane.prep_state = False
        events: list = []
        for name in ("afc_vivid:stage_read_begin", "afc_vivid:stage_read_end"):
            unit.printer.register_event_handler(
                name, lambda staged, name=name: events.append((name, staged)))

        result = unit._stage_and_load(lane)

        assert result == (False, 0.0)
        assert lane.move_to.call_args_list == []
        assert events == [("afc_vivid:stage_read_begin", lane),
                          ("afc_vivid:stage_read_end", lane)]
        assert unit.logger.messages == []

    def test_a_lane_already_at_the_sensor_feeds_nothing(self):
        unit = _build_vivid_unit()
        lane = _build_vivid_lane()
        lane.raw_load_state = True
        events: list = []
        for name in ("afc_vivid:stage_read_begin", "afc_vivid:stage_read_end"):
            unit.printer.register_event_handler(
                name, lambda staged, name=name: events.append((name, staged)))

        result = unit._stage_and_load(lane)

        assert result == (True, 0.0)
        assert lane.move_to.call_args_list == []
        assert events == [("afc_vivid:stage_read_begin", lane),
                          ("afc_vivid:stage_read_end", lane)]
        assert unit.logger.messages == []

    def test_a_real_miss_is_retried_once(self):
        unit = _build_vivid_unit()
        lane = _build_vivid_lane()
        lane.move_to.return_value = (False, 50.0, AFCMoveWarning.ERROR)

        result = unit._stage_and_load(lane)

        assert result == (False, 100.0)
        assert lane.move_to.call_args_list == [self.CALIBRATED_FEED, self.CALIBRATED_FEED]
        assert unit.logger.messages == []

    def test_early_stops_end_after_eight_segments(self):
        # Each feed "homes" (an RFID detect) without the load sensor tripping.
        unit = _build_vivid_unit()
        lane = _build_vivid_lane()
        lane.move_to.return_value = (True, 30.0, AFCMoveWarning.NONE)

        result = unit._stage_and_load(lane)

        assert result == (False, 240.0)
        assert lane.move_to.call_args_list == [self.CALIBRATED_FEED] * 8
        assert unit.logger.messages == []

    def test_an_uncalibrated_lane_feeds_the_calibration_budget_slowly(self):
        unit = _build_vivid_unit()
        lane = _build_vivid_lane()
        lane.calibrated_lane = False
        self._feed_trips_sensor(lane, 1234.5)

        result = unit._stage_and_load(lane)

        assert result == (True, 1234.5)
        assert lane.move_to.call_args_list == [
            mock.call(5000, SpeedMode.SHORT, assist_active=AssistActive.NO,
                      endstop="lane1_load", use_homing=True)]
        assert unit.logger.messages == []


# ── eject_lane ────────────────────────────────────────────────────────────────

class TestEjectLane:
    def test_calls_select_and_unselect(self):
        unit = _make_vivid()
        lane = MagicMock()
        lane.dist_hub = 200.0
        unit.select_lane = MagicMock()
        unit.unselect_lane = MagicMock()

        unit.eject_lane(lane)

        unit.select_lane.assert_called_once_with(lane)
        unit.unselect_lane.assert_called_once()

    def test_disables_steppers(self):
        unit = _make_vivid()
        lane = MagicMock()
        lane.dist_hub = 200.0
        unit.select_lane = MagicMock()
        unit.unselect_lane = MagicMock()

        unit.eject_lane(lane)

        unit.selector_stepper_obj.do_enable.assert_called_with(False)
        unit.drive_stepper_obj.do_enable.assert_called_with(False)

    def test_adjusts_distance_when_dist_hub_over_400(self):
        from extras.AFC_lane import MoveDirection
        unit = _make_vivid()
        lane = MagicMock()
        lane.hub_obj = MagicMock()
        lane.hub_obj.hub_clear_move_dis = 65
        lane.dist_hub = 600.0  # > 400 → should subtract LANE_OVERSHOOT+100
        unit.select_lane = MagicMock()
        unit.unselect_lane = MagicMock()

        unit.eject_lane(lane)

        expected_dist = (600.0 - (AFC_vivid.LANE_OVERSHOOT + 100) - \
                         lane.hub_obj.hub_clear_move_dis - lane.homing_overshoot) * MoveDirection.NEG
        call_args = lane.move_to.call_args[0]
        assert call_args[0] == expected_dist

    def test_does_not_adjust_distance_when_dist_hub_at_or_below_400(self):
        from extras.AFC_lane import MoveDirection
        unit = _make_vivid()
        lane = MagicMock()
        lane.dist_hub = 300.0  # <= 400 → full dist used
        unit.select_lane = MagicMock()
        unit.unselect_lane = MagicMock()

        unit.eject_lane(lane)

        expected_dist = 300.0 * MoveDirection.NEG
        call_args = lane.move_to.call_args[0]
        assert call_args[0] == expected_dist

    def test_treats_hub_clear_move_dis_as_zero_when_no_hub_obj(self):
        """The hub_clear_move_dis ternary takes its False branch (0) when
        lane.hub_obj is None, rather than raising on hub_obj.hub_clear_move_dis."""
        from extras.AFC_lane import MoveDirection
        unit = _make_vivid()
        lane = MagicMock()
        lane.hub_obj = None
        lane.dist_hub = 600.0  # > 400 → should subtract LANE_OVERSHOOT+100 only
        unit.select_lane = MagicMock()
        unit.unselect_lane = MagicMock()

        unit.eject_lane(lane)

        expected_dist = (600.0 - (AFC_vivid.LANE_OVERSHOOT + 100) - \
                         lane.homing_overshoot) * MoveDirection.NEG
        call_args = lane.move_to.call_args[0]
        assert call_args[0] == expected_dist
