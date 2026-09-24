"""
Typed test fakes for the OpenAMS unit tests.

These replace broad MagicMock usage: every fake declares its real attribute
set explicitly, so a typo'd or missing attribute fails the test instead of
silently returning a truthy MagicMock. Callables under observation are
Recorder instances (plain callables that record their calls).

Self-contained on purpose: the OpenAMS tests must not import from the ACE
test helpers so the OpenAMS work can be upstreamed on its own.
"""

from __future__ import annotations


class Recorder:
    """A plain callable that records calls; optionally returns a fixed value
    or raises. Replaces MagicMock for callbacks/methods under observation."""

    def __init__(self, result=None, raises=None):
        self.calls = []
        self.result = result
        self.raises = raises

    def __call__(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        if self.raises is not None:
            raise self.raises
        return self.result

    @property
    def called(self):
        return len(self.calls) > 0

    @property
    def call_count(self):
        return len(self.calls)

    @property
    def last_args(self):
        return self.calls[-1][0]

    @property
    def last_kwargs(self):
        return self.calls[-1][1]


class FakeLogger:
    """Records log lines per level."""

    def __init__(self):
        self.lines = {"info": [], "debug": [], "warning": [], "error": []}

    # AFC'S SIGNATURES, NOT THE STDLIB'S. These used to take (msg, *a, **k),
    # which accepts printf-style args -- and so passed calls the real AFC
    # logger rejects. Two of those were live in AFC_ACE2: an autodetect
    # info() that would have raised TypeError, and a debug() whose cmd bound
    # to only_debug while the message printed its own %d. A fake that is
    # laxer than the real thing is how a call like that reaches hardware.
    def info(self, msg, console_only=False):
        self.lines["info"].append(msg)

    def debug(self, msg, only_debug=False, traceback=None):
        self.lines["debug"].append(msg)

    def warning(self, msg):
        self.lines["warning"].append(msg)

    def error(self, msg, traceback=None, stack_name=""):
        self.lines["error"].append(msg)


class FakeReactor:
    NOW = 0.0
    NEVER = 9_999_999_999.0

    def __init__(self, monotonic_value=100.0):
        self._monotonic = monotonic_value
        self.register_callback = Recorder()
        self.register_timer = Recorder(result="timer-handle")
        self.unregister_timer = Recorder()
        self.update_timer = Recorder()

    def monotonic(self):
        return self._monotonic

    def pause(self, until):
        pass

    def completion(self):
        comp = Recorder()
        comp.complete = Recorder()
        return comp


class FakeError:
    """Stands in for afc.error: records AFC_error calls."""

    def __init__(self):
        self.AFC_error = Recorder()


class FakeFunction:
    """Stands in for afc.function with explicit printing/pause state."""

    def __init__(self, printing=False, paused=False, in_print_flag=False):
        self.printing = printing
        self.paused = paused
        self.in_print_flag = in_print_flag
        self.raise_on_is_printing = None

    def is_printing(self, check_movement=False):
        if self.raise_on_is_printing is not None:
            raise self.raise_on_is_printing
        return self.printing

    def is_paused(self):
        return self.paused

    def in_print(self):
        return self.in_print_flag


class FakeAFC:
    """Explicit-attribute afc object for unit tests."""

    def __init__(self):
        # Jimmy's led_effects work (led_effects branch) reads and clears this
        # on every lane-state change; the shared conftest MockAFC gained it in
        # the same commit, but these helpers roll their own fake.
        self.active_led_effects = []
        self.lanes = {}
        self.current = None
        self.in_toolchange = False
        self.error = FakeError()
        self.function = FakeFunction()
        self.reactor = FakeReactor()
        self.logger = FakeLogger()
        self.save_vars = Recorder()
        self.load_to_hub = False


class FakeExtruderObj:
    """AFC_extruder stand-in: section name, physical name, loaded lane."""

    def __init__(self, name="extruder", th_extruder_name=None, lane_loaded=None):
        self.name = name
        self.th_extruder_name = th_extruder_name if th_extruder_name is not None else name
        self.lane_loaded = lane_loaded


class FakeLane:
    """AFCLane stand-in with the explicit state the ACE/OpenAMS units touch."""

    def __init__(self, name, extruder_obj=None, hub_obj=None,
                 tool_loaded=False, runout_lane=None, status=None):
        self.name = name
        self.extruder_obj = extruder_obj
        self.hub_obj = hub_obj
        self.tool_loaded = tool_loaded
        self.runout_lane = runout_lane
        self.status = status
        self.prep_state = False
        self.loaded_to_hub = False
        self._load_state = False
        self._load_suppressed = False
        self.current_led_state = ""
        self._afc_prep_done = True
        self._oams_runout_detected = False
        self._oams_runout_empty = False
        self.load_to_hub = False
        self.use_feed_assist = None
        # Upstream's virtual-tools work made a lane's map a LIST of T(n)
        # commands, rendered by map_to_string(); the unit modules print it in
        # their prep line. Mirrors AFCLane._format_map: sorted and comma-joined,
        # "NONE" when nothing is mapped.
        self.map = None
        # Observable lane methods
        self.handle_load_runout = Recorder()
        self.sync_to_extruder = Recorder()
        self.unsync_to_extruder = Recorder()
        self.enable_buffer = Recorder()
        self.set_tool_unloaded = Recorder()
        self.get_toolhead_pre_sensor_state = Recorder(result=False)

    def map_to_string(self):
        if not self.map:
            return "NONE"
        if isinstance(self.map, str):
            return self.map
        return ", ".join(sorted(self.map))


class FakeOams:
    """OAMS hardware stand-in for sensor polling and status queries."""

    def __init__(self, f1s=None, hub=None, encoder_clicks=0, current_spool=None,
                 fps_value=0.0, action_status=None, load_failures=0,
                 unload_failures=0):
        self.f1s_hes_value = f1s if f1s is not None else [0, 0, 0, 0]
        self.hub_hes_value = hub if hub is not None else [0, 0, 0, 0]
        self.encoder_clicks = encoder_clicks
        self.current_spool = current_spool
        self.fps_value = fps_value
        self.action_status = action_status
        self.load_retry_failures = load_failures
        self.unload_retry_failures = unload_failures
