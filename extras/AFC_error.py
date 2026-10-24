# AFCProject Automated Filament Changer Software
#
# Copyright (C) 2024-2026 Armored Turtle
# Copyright (C) 2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import traceback
import logging
import inspect

from configparser import Error as error

from typing import TYPE_CHECKING, Any, Optional

if TYPE_CHECKING:
    from klippy import Printer
    from configfile import ConfigWrapper
    from gcode import GCodeCommand
    from extras.AFC import afc
    from extras.AFC_lane import AFCLane
    from extras.pause_resume import PauseResume
    from extras.idle_timeout import IdleTimeout

try: from extras.AFC_utils import ERROR_STR
except:
    err_str = f"Error when trying to import AFC_utils.ERROR_STR\n{traceback.format_exc()}"
    raise error(err_str)

try: from extras.AFC import State
except:
    err_str = ERROR_STR.format(import_lib="AFC", trace=traceback.format_exc())
    raise error(err_str)

try: from extras.AFC_lane import AFCLaneState, MoveDirection, SpeedMode
except:
    err_str = ERROR_STR.format(import_lib="AFC_lane", trace=traceback.format_exc())
    raise error(err_str)

try: from extras.AFC_functions import get_gcode_absolute_extrude, set_gcode_absolute_extrude
except:
    err_str = ERROR_STR.format(import_lib="AFC_functions", trace=traceback.format_exc())
    raise error(err_str)

class afcError:
    def __init__(self, config: ConfigWrapper) -> None:
        """
        Register for the klippy:connect event; real setup happens in handle_connect
        since AFC and other printer objects aren't available yet at this point.

        :param config: Klipper config wrapper for the AFC_error section
        """
        self.printer: Printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.errorLog: dict[str, Any] = {}
        self.pause= False
        # Coordinate and extrude modes from before AFC_PAUSE forced absolute mode
        self.pause_modes: Optional[tuple[bool, bool]] = None

    def handle_connect(self) -> None:
        """
        Handle the connection event.
        This function is called when the printer connects. It looks up AFC info
        and assigns it to the instance variable `self.AFC`.
        """
        self.afc: afc       = self.printer.lookup_object('AFC')
        self.pause_resume: PauseResume = self.printer.lookup_object("pause_resume")
        self.logger         = self.afc.logger
        self.error_timeout  = self.afc.error_timeout
        self.idle_timeout_obj: IdleTimeout = self.printer.lookup_object("idle_timeout")
        self.idle_timeout_val = self.idle_timeout_obj.idle_timeout

        # Constant variable for renaming RESUME macro
        self.BASE_RESUME_NAME       = 'RESUME'
        self.AFC_RENAME_RESUME_NAME = f'_AFC_RENAMED_{self.BASE_RESUME_NAME}_'
        self.BASE_PAUSE_NAME        = 'PAUSE'
        self.AFC_RENAME_PAUSE_NAME  = f'_AFC_RENAMED_{self.BASE_PAUSE_NAME}_'

        self.afc.gcode.register_command('RESET_FAILURE', self.cmd_RESET_FAILURE, desc=self.cmd_RESET_FAILURE_help)
        self.afc.gcode.register_command('AFC_RESUME', self.cmd_AFC_RESUME, desc=self.cmd_AFC_RESUME_help)

    def fix(self, problem: Optional[str], lane: Optional[str|AFCLane]) -> bool:
        """
        Attempt to resolve a reported lane/toolhead problem automatically,
        falling back to pausing for user intervention when it can't be handled.

        :param problem: identifier for the problem to resolve, or None for an unknown error
        :param lane: lane object, or the name of a lane to look up, that the problem applies to
        :return type: True if the problem was resolved automatically, False otherwise
        """
        self.pause= True
        error_handled = False
        lane_name = lane if isinstance(lane, str) else None
        if isinstance(lane, str):
            lane = self.afc.lanes.get(lane, None)
            if lane is None:
                self.PauseUserIntervention(
                    f"Unknown lane '{lane_name}' reported for problem: {problem}"
                )
                return error_handled

        if problem is None:
            self.PauseUserIntervention('Paused for unknown error')
        elif(problem=='toolhead'
            and lane is not None):
            error_handled = self.ToolHeadFix(lane)
        else:
            self.PauseUserIntervention(problem)
        if (not error_handled
            and lane is not None):
            lane.unit_obj.lane_fault(lane)

        return error_handled

    def ToolHeadFix(self, cur_lane: AFCLane) -> bool:
        """
        Attempt to automatically resolve a toolhead-related lane fault by
        retracting or reloading filament to the lane's load sensor.

        :param cur_lane: lane object to attempt to fix
        :return type: True if the lane was successfully reset, False otherwise
        """
        if cur_lane.get_toolhead_pre_sensor_state():   #toolhead has filament
            if cur_lane.extruder_obj.lane_loaded == cur_lane.name:   #var has right lane loaded
                if not cur_lane.raw_load_state: #Lane has filament
                    self.PauseUserIntervention('Filament not loaded in Lane')
                else:
                    self.PauseUserIntervention('no error detected')
            else:
                self.PauseUserIntervention('laneloaded does not match extruder')

        else: #toolhead empty
            failed_to_retract_msg = f"Failed to retract {cur_lane.name} to load sensor"
            if (cur_lane.raw_load_state
                and not cur_lane.is_direct_hub()
                and cur_lane.extruder_obj.tool_start != "buffer"):
                self.logger.info(f"Retracting {cur_lane.name} back to load switch")
                if self.afc.homing_enabled:
                    num_tries = 0
                    while (cur_lane.raw_load_state):
                        total_move_dist = cur_lane.dist_hub + 500
                        if cur_lane.hub_obj is not None:
                            total_move_dist += cur_lane.hub_obj.afc_bowden_length
                        cur_lane.unit_obj.move_to_load(cur_lane, total_move_dist,
                                                       MoveDirection.NEG, True,
                                                       SpeedMode.SHORT)
                        num_tries += 1
                        if num_tries >= 5:
                            self.PauseUserIntervention(failed_to_retract_msg)
                            return False
                else:
                    max_length = 5000
                    while cur_lane.raw_load_state:  # slowly back filament up to lane extruder
                        cur_lane.move(-5, self.afc.short_moves_speed, self.afc.short_moves_accel, True)
                        if max_length > 0:
                            max_length -= 5
                        else:
                            self.PauseUserIntervention(failed_to_retract_msg)
                            return False
                    max_length = 1000
                    while not cur_lane.raw_load_state:  # reload lane extruder
                        cur_lane.move(5, self.afc.short_moves_speed, self.afc.short_moves_accel, True)
                        if max_length > 0:
                            max_length -= 5
                        else:
                            self.PauseUserIntervention(
                                f"Failed to move back {cur_lane.name} to load sensor"
                            )
                            return False

                cur_lane.set_tool_unloaded()
                cur_lane.loaded_to_hub = False
                cur_lane.unit_obj.prep_load(cur_lane)
                cur_lane.unit_obj.prep_post_load(cur_lane)
                self.afc.save_vars()
                self.pause = False
                self.logger.info(f"Done resetting {cur_lane.name}")
                return True
        return False

    def PauseUserIntervention(self, message: str) -> None:
        """
        Log an error message and pause the print for user intervention if the
        printer is homed, not already paused, and a pause has been requested.

        :param message: error message to log
        """
        self.logger.error(message)
        if (self.afc.function.is_homed(for_move=True)
            and not self.afc.function.is_paused()):
            self.afc.save_pos()
            if self.pause:
                self.pause_print()

    def pause_print(self) -> None:
        """
        pause_print function verifies that the printer is homed and not currently paused before calling
        the base pause command
        """
        self.set_error_state( True )
        self.logger.info ('PAUSING')
        self.afc.gcode.run_script_from_command('PAUSE')
        self.logger.debug("After User Pause")
        self.afc.function.log_toolhead_pos()

    def set_error_state(self, state: bool=False) -> None:
        """
        Set the AFC error state, saving the toolhead position on the first
        transition into an error state.

        :param state: True to enter the error state, False to clear it
        """
        logging.warning(f"AFC debug: setting error state {state}")
        # Only save position on first error state call
        if (state
            and not self.afc.error_state):
            self.afc.save_pos()
        self.afc.error_state = state
        self.afc.current_state = State.ERROR if state else State.IDLE

    def AFC_error(self, msg: str, pause: bool=True, stack_name: Optional[str]=None) -> None:
        """
        Log an AFC error and optionally pause the print.

        :param msg: error message to log
        :param pause: True to pause the print after logging
        :param stack_name: name to attribute the error to, defaults to the caller's function name
        """
        # Print to logger since respond_raw does not write to logger
        logging.warning(msg)
        if stack_name is None:
            frame=inspect.currentframe()
            caller_frame = frame.f_back if frame else None
            stack_name = caller_frame.f_code.co_name if caller_frame else ""
        # Handle AFC errors
        self.logger.error(message=msg, stack_name=stack_name)
        if pause: self.pause_print()

    cmd_RESET_FAILURE_help = "CLEAR STATUS ERROR"
    def cmd_RESET_FAILURE(self, gcmd: GCodeCommand) -> None:
        """
        This function clears the error state of the AFC system by setting the error state to False.

        Usage
        -----
        `RESET_FAILURE`

        Example
        -----
        `RESET_FAILURE`
        """
        self.reset_failure()

    def reset_failure(self) -> None:
        """
        Common function to reset error_state, pause, and position_saved variables
        """
        self.logger.debug("Resetting failures")
        self.set_error_state(False)
        self.pause              = False
        self.afc.position_saved = False
        self.afc.in_toolchange  = False

    cmd_AFC_RESUME_help = "Clear error state and restores position before resuming the print"
    def cmd_AFC_RESUME(self, gcmd: GCodeCommand) -> None:
        """
        During the PREP phase of startup, the user's RESUME macro is renamed and replaced with AFC_RESUME.
        This function clears the error state of the AFC system, sets the in_toolchange flag to False,
        runs the resume script, and restores the toolhead position to the last saved position.

        A tool change that failed part way during the print is finished first, and the print stays
        paused if that load fails again. When the user's RESUME leaves the print paused, the saved
        position is kept for the next resume.

        This is not a macro that should normally need to be called by the user.

        Usage
        -----
        `AFC_RESUME`

        Example
        -----
        ```
        AFC_RESUME
        ```
        """
        self.afc.in_toolchange = False
        if not self.afc.function.is_paused():
            self.logger.debug("AFC_RESUME: Printer not paused, not executing resume code")
            return

        # Save current pause state
        temp_is_paused = self.afc.function.is_paused()

        if not self._finish_interrupted_toolchange():
            return

        # Verify that printer is in absolute mode
        self.afc.function.check_absolute_mode("AFC_RESUME")

        move_z_pos = self.afc.last_gcode_position[2] + self.afc.z_hop
        # Check if current position is below saved gcode position, if its lower first raise z above last saved
        #   position so that toolhead does not crash into part
        if (self.afc.function.is_homed(for_move=True)
            and self.afc.gcode_move.last_position[2] <= move_z_pos):
            self.afc.move_z_pos(move_z_pos, "AFC_RESUME")
        else:
            self.logger.debug(f"AFC_RESUME: not moving in z cur_pos:{self.afc.gcode_move.last_position} move_z_pos:{move_z_pos}")

        self.logger.debug("AFC_RESUME: Before User Restore")
        self.afc.function.log_toolhead_pos()
        self.afc.gcode.run_script_from_command(
            f"{self.AFC_RENAME_RESUME_NAME} {gcmd.get_raw_command_parameters()}"
        )

        # The user's RESUME can refuse and leave the print paused (a Snapmaker U1
        # does when its toolhead sensor reads empty). Keep the saved position and
        # modes for the resume that does go through.
        if self.afc.function.is_paused():
            self.logger.info("AFC_RESUME: the print is still paused, keeping the saved "
                             "position for the next resume")
            return

        # The only time our resume should restore position is if there was an error that caused the pause
        if (self.afc.error_state
            or temp_is_paused
            or self.afc.position_saved):
            self.set_error_state(False)
            # Restore position if the printer is homed, otherwise leave it where it is
            if self.afc.function.is_homed(for_move=True):
                self.afc.restore_pos(False)
            self.pause = False

        # restore_pos may not run (a tool change during the pause already used the saved
        # position), and the user's RESUME brings back the absolute modes AFC_PAUSE set
        if self.pause_modes is not None:
            absolute_coord, absolute_extrude = self.pause_modes
            self.afc.gcode_move.absolute_coord = absolute_coord
            set_gcode_absolute_extrude(self.afc.gcode_move, absolute_extrude)
            self.pause_modes = None
            self.logger.debug(
                f"AFC_RESUME: restored absolute_coord {absolute_coord}, "
                f"absolute_extrude {absolute_extrude}"
            )

        self.logger.debug(
            f"RESUME-Error State: {self.afc.error_state}, "
            f"Is Paused {self.afc.function.is_paused()}, "
            f"Position_saved {self.afc.position_saved}, "
            f"in toolchange: {self.afc.in_toolchange}"
        )

    def _finish_interrupted_toolchange(self) -> bool:
        """
        Finish the tool change a failure interrupted, before the user's RESUME.

        CHANGE_TOOL records the T(n) and lane it was loading when its unload or load
        fails and the print pauses. The print file has already moved past that T(n),
        so without this a plain RESUME prints with the old lane, or an empty toolhead.
        A lane loaded by hand for that T(n) during the pause, or filament in the
        bypass, needs nothing more.

        :return bool: True to go on with the resume, False to stay paused
        """
        pending = self.afc.resume_toolchange
        if pending is None:
            return True
        tool, lane_name, purge_length, filename = pending
        in_print, print_file = self.afc.function.in_print(return_file=True)
        # A SET_MAP during the pause moves the T(n) to another lane
        lane = self.afc.lanes.get(self.afc.tool_cmds.get(tool, lane_name))
        if (not in_print
            or print_file != filename
            or lane is None
            or lane.tool_loaded
            or self.afc.get_bypass_state()):
            self.afc.resume_toolchange = None
            return True

        self.logger.info(f"AFC_RESUME: finishing the tool change to {lane.name}")
        self._home_xy_if_lost()
        self.set_error_state(False)
        # The pause's own position is restored after the user's RESUME, and the
        # change was counted when it first ran
        self.afc.CHANGE_TOOL(lane, purge_length, restore_pos=False, count_change=False)
        if self.afc.error_state or not lane.tool_loaded:
            self.logger.error(f"AFC_RESUME: {lane.name} did not load, so the print stays "
                              "paused. Clear the fault and resume again.")
            return False
        return True

    def _home_xy_if_lost(self) -> None:
        """
        Home X and Y before finishing a tool change when only they are unhomed.

        A Snapmaker U1 drops the X and Y homing when a head fails to dock, and the
        park and pick that finish the change need them. Only a printer with auto_home
        homes here, as in check_homed. Z is never homed mid print, so with it unhomed
        as well the change's own homing check decides.
        """
        if not self.afc.auto_home:
            return
        curtime = self.afc.reactor.monotonic()
        homed = self.afc.toolhead.get_kinematics().get_status(curtime)["homed_axes"]
        if "z" in homed and ("x" not in homed or "y" not in homed):
            self.logger.info("AFC_RESUME: homing X and Y before finishing the tool change")
            self.afc.gcode.run_script_from_command("G28 X Y")
            self.afc.toolhead.wait_moves()

    def point_snapmaker_resume_at(self, lane: AFCLane) -> None:
        """
        Tell a Snapmaker U1's INNER_RESUME which head a load during a pause left to resume on.

        INNER_PAUSE saves the extruder that was active at the pause, and INNER_RESUME
        re-selects, heats and primes that one, which would swap a head loaded during the
        pause (AFC_RESUME finishing an interrupted change, or a T(n) or TOOL_LOAD by hand)
        back out. The resume temperature is the highest of the slicer's temperature for
        the lane's T(n), the target INNER_PAUSE saved for that head and the heater's
        target, so a load's material default never lowers the print's temperature.
        Printers without the macro, Snapmaker or not, need nothing.

        :param lane: Lane the load put in the head
        """
        if not self.afc.snapmaker_printer:
            return
        macro = self.printer.lookup_object("gcode_macro INNER_RESUME", None)
        if macro is None:
            return
        index = lane.lane_extruder_index
        candidates = [lane.extruder_obj.get_heater().target_temp,
                      macro.variables.get(f"extruder{index}_temp", 0)]
        try:
            tool = int(str(lane.current_map).lstrip("T"))
            if tool >= 0:
                candidates.append(self.afc.print_tool_temperatures[tool])
        except (ValueError, IndexError, TypeError):
            pass
        target = max(float(temp) for temp in candidates)
        self.logger.debug(f"AFC_RESUME: INNER_RESUME now resumes on "
                          f"{lane.extruder_obj.th_extruder_name} at {target:.0f}C")
        for variable, value in (("last_extruder_index", index),
                                ("last_extruder_temp", target),
                                (f"extruder{index}_temp", target)):
            self.afc.gcode.run_script_from_command(
                f"SET_GCODE_VARIABLE MACRO=INNER_RESUME VARIABLE={variable} VALUE={value}")

    cmd_AFC_PAUSE_help = "Pauses print, raises z by z-hop amount, and then calls users pause macro"
    def cmd_AFC_PAUSE(self, gcmd: GCodeCommand) -> None:
        """
        During the PREP phase of startup, the user's PAUSE macro is renamed and replaced with AFC_PAUSE.
        This function pauses the print, raises the Z axis by the z-hop amount, and then calls the user's pause macro.

        This is not a macro that should normally need to be called by the user.

        Usage
        -----
        `AFC_PAUSE`

        Example
        -----
        ```
        AFC_PAUSE
        ```
        """
        # Check to make sure printer is not already paused
        if not self.afc.function.is_paused():
            self.logger.debug("AFC_PAUSE: Pausing")
            # Save position
            self.afc.save_pos()
            # The user's PAUSE saves its state after the switch to absolute mode below, so
            # keep the print's own modes for AFC_RESUME to put back. A tool change that
            # pauses part way has already switched to absolute mode, so take the modes
            # its saved position holds from before that.
            if self.afc.position_saved:
                self.pause_modes = (self.afc.absolute_coord, self.afc.absolute_extrude)
            else:
                self.pause_modes = (self.afc.gcode_move.absolute_coord,
                                    get_gcode_absolute_extrude(self.afc.gcode_move))
            # Need to pause as soon as possible to stop more gcode from executing, this needs to be done before movement in Z
            self.pause_resume.send_pause_command()
            # Verify that printer is in absolute mode
            self.afc.function.check_absolute_mode("AFC_PAUSE")
            move_z_pos = self.afc.last_gcode_position[2] + self.afc.z_hop
            # Check to see if current position is less than saved position plus z-hop
            if (self.afc.function.is_homed(for_move=True)
                and self.afc.gcode_move.last_position[2] <= move_z_pos):
                # Move Z up by z-hop value
                self.afc.move_z_pos(move_z_pos, "AFC_PAUSE")
            else:
                self.logger.debug(f"AFC_PAUSE: not moving in z cur_pos:{self.afc.gcode_move.last_position} move_z_pos:{move_z_pos}")
            # Call users PAUSE
            self.afc.gcode.run_script_from_command(
                f"{self.AFC_RENAME_PAUSE_NAME} {gcmd.get_raw_command_parameters()}"
            )

            timeout_to_use = max(self.error_timeout, self.idle_timeout_val)
            self.afc.gcode.run_script_from_command(f"SET_IDLE_TIMEOUT TIMEOUT={timeout_to_use}")

        else:
            self.logger.debug("AFC_PAUSE: Not Pausing")

        self.logger.debug(
            f"PAUSE-Error State: {self.afc.error_state}, "
            f"Is Paused {self.afc.function.is_paused()}, "
            f"Position_saved {self.afc.position_saved}, "
            f"in toolchange: {self.afc.in_toolchange}"
        )


    handle_lane_failure_help = "Get load errors, stop stepper and respond error"
    def handle_lane_failure(self, cur_lane: AFCLane, message: str, pause: bool=True) -> None:
        """
        Disable a lane's stepper, mark it as errored, and report the failure.

        :param cur_lane: lane object that failed
        :param message: description of the failure
        :param pause: True to pause the print after reporting the failure
        """
        # Disable the stepper for this lane
        cur_lane.do_enable(False)
        cur_lane.status = AFCLaneState.ERROR
        msg = f"{cur_lane.name} {message}"
        frame=inspect.currentframe()
        caller_frame = frame.f_back if frame else None
        stack_name = caller_frame.f_code.co_name if caller_frame else ""

        self.AFC_error(msg, pause, stack_name=stack_name)
        cur_lane.unit_obj.lane_fault(cur_lane)

def load_config(config: ConfigWrapper) -> afcError:
    """
    Klipper config entry point for the AFC_error module.

    :param config: Klipper config wrapper for the AFC_error section
    :return type: afcError instance to register with the printer
    """
    return afcError(config)