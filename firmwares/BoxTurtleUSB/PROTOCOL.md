# boxturtle-1 protocol

The AFC-Lite running this firmware is a USB CDC serial device
(`/dev/serial/by-id/usb-AFC_BoxTurtle_<uid>-if00`). Both directions carry one
flat JSON object per line. The firmware ends its lines with `\n`; it takes
`\n` or `\r` from the host and skips anything before the first `{` of a
line, so a stray probe (a modem manager's `AT\r`, another program's frames)
does not spoil the next request. Lines longer than 255 bytes are dropped.

The USB interface does not claim the modem (AT) protocol, so ModemManager
leaves the port alone without a udev rule. The host should still open the
port exclusively (`TIOCEXCL`), since other programs may probe serial ports.

Numbers may have decimals; the firmware keeps three (thousandths), so `12.5`
mm is exact. Lanes are numbered 1 to 4.

## Requests and replies

Every request has `"cmd"` and may have `"id"`. Every reply echoes the `id`
and carries `"ok"`; a failed request also carries `"err"`.

```
> {"id":1,"cmd":"info"}
< {"id":1,"ok":true,"name":"AFC-Turtle","fw":"0.1.0","proto":"boxturtle-1","board":"afc-lite","lanes":4,"uid":"...","reset":"power_on","up":5123,"dropped":0,"rx":1,"tx":0,"trial":false,"boot_fails":0}
```

`reset` is why the board last started: `power_on`, `reset_pin`, `software`,
`watchdog`, `brownout`, `low_power`, `unknown`. `rx` and `tx` count the lines
received and sent since boot, `dropped` the lines too long to take. `trial`
and `boot_fails` are the boot guard's state (see Boot guard). Send a `\n`
before the first request after opening the port, so a half line left in the
board's buffer is ended.

Lines with `ev` or `ok` and no `cmd` (the board's own lines echoed back) are
ignored.

| cmd | fields | what it does |
|---|---|---|
| `info` | | identity, version, reset cause, uptime (ms) |
| `ping` | | keeps the link alive (see link loss) |
| `status` | | `sw` (every switch), `lanes` (pos, v, en, tmc, job, espool, assist, follow: 0 off or `tn`, 1 `follow`, 2 `ext`) |
| `move` | `lane`, `mm` (signed, + toward the toolhead), `speed`, `accel`, `until`, `until_v`, `watch`, `watch_v`, `hold`, `espool_fwd`, `espool_rwd`, `espool_kick`, `seq` | one move; ends early when `until` reaches `until_v` (default 1), or when `watch` leaves `watch_v` (default 1). Replies at once; `move_done` follows. The espool keys set the espooler duty while this move feeds or retracts (see Espoolers) |
| `stop` | `lane` (optional) | stops moves, jobs, assist and espoolers of one or all lanes |
| `enable` | `lane`, `on` | driver enable; `on:false` is ignored while the lane moves |
| `job` | see Jobs | a self-recovering sequence |
| `assist` | `lane`, `on`, `mode` (`tn` default, `follow`, `ext`), `espool_fwd`, `espool_rwd`, `espool_kick` | `tn`: TurtleNeck feed assist while printing: the lane tracks the extruder position both ways, each change scaled exactly like a lane synced to the extruder under AFC_buffer: `assist_low` after the advance switch changes (pressed or released), `assist_high` after the trailing switch changes, and `assist_high` before either has changed since power up. Each multiplier holds until the other switch changes, so the TurtleNeck sweeps between its switches. While `assist_fault_mm` is above 0 (AFC's fault detection) a pressed switch scales harder, advance `assist_low` x 2/5 and trailing `assist_high` x 1.5, and a start uses the harder one for the last switch until a switch changes. The lane stops when the extruder stops. `follow`: the lane tracks the extruder position both ways (AFC's sync_to_extruder during a load or unload), no faults; the espooler follows the lane. `ext`: printing with a buffer the host reads (an AFC_buffer on a Klipper MCU): the lane tracks the position the host sends, already scaled by that buffer's multiplier, with print assist pulses; the host's buffer checks for faults. Sending it again switches modes. Every start waits for a new `e` before it moves |
| `e` | `pos`, `lane` | the position the lane follows in mm (the extruder's, or in `ext` mode the host's scaled one), sent a few times a second while assist runs. With no `lane` it goes to every lane. No reply |
| `espool` | `lane`, `duty` (-1..1), `kick` | sets an espooler by hand; 0 brakes and hands it back to the automatic mode |
| `lane_cfg` | `lane`, `steps_per_mm`, `dir_invert`, `run_current`, `hold_current`, `sense_resistor`, `microsteps`, `stealthchop`, `sgthrs`, `espool_pulse_mm`, `espool_pulse_ms` | lane and driver settings; the pulse keys are this lane's print assist pulses (negative = back to the `cfg` value) |
| `cfg` | see Settings | unit-wide settings |
| `sw_cfg` | `sw`, `invert`, `debounce_ms` | switch polarity and debounce |
| `led_cfg` | `strip` (1-4 = RGB1-RGB4), `count`, `order` (`GRB`, `GRBW`, `RGB`, `RGBW`) | LED string layout; every header has no LEDs until this sets it |
| `led` | `strip`, `i` (first, 1-based), `n`, `color` (`RRGGBB` or `RRGGBBWW`), `send` | sets LEDs; `send:false` holds the update for the next `led` |
| `tmc_read` | `lane`, `reg` | raw driver register as `hi` and `lo` 16-bit halves |
| `bootloader` | | reboots into the bootloader (Katapult or DFU) |
| `reboot` | | reboots |
| `tmc_wtest` | `lane`, `reg`, `hi`, `lo`, `n`, `gap`, `flags` | bench: writes the register `n` times with `gap` idle bit times before each datagram, replies the driver's write counter (IFCNT) `before` and `after` |
| `rxlog` | | bench: the first 20 bytes (hex) of the last 6 lines received, as `lines` |
| `replay` | `lane` | sends the lane's last `move_done` and `job_done` again, then replies (for a host that lost them in a link drop) |

Switch names: `hub`, `prep1`..`prep4`, `load1`..`load4`, `tn_adv`,
`tn_trl`. In `move` and `job`, `prep` and `load` mean the lane's own switch.

## Events

Sent unasked:

| ev | fields | when |
|---|---|---|
| `sw` | `sw`, `v` | a switch changed (after debounce) |
| `move_done` | `lane`, `end`, `mm`, `seq` | a `move` ended. `end`: `done`, `triggered`, `no_trigger` (full distance, `until` never reached), `watch` (the watch switch left its state), `stopped`, `fault` (driver), `link_lost` (stopped on link loss; the driver keeps holding). `seq` is the move's own, when it had one |
| `job` | `lane`, `job`, `state` (`started`, `recovering`), `try`, `why` | a job started or is retrying on its own |
| `job_done` | `lane`, `job`, `ok`, `tries`, `mm`, `seq`, `code`, `msg` | a job finished; `ok:false` comes with a `fault` too |
| `fault` | `code`, `lane`, `msg` | see Faults |
| `log` | `msg` | information |

## Jobs

`{"cmd":"job","job":"feed","lane":1,"until":"hub","mm":250,"speed":100,"watch":"prep"}`

- `feed`: moves toward the toolhead until `until` triggers, at most `mm`
  per try. No trigger: back off `backoff_mm` (20), retry at `slow_pct` (50 %)
  of the speed. With `min_mm`, a trigger before `min_mm` is a snag: back off
  30 mm and retry. A `watch` switch that opens (runout) fails at once.
- `unload`: retracts until `until` (default `hub`) clears, then `extra_mm`
  more. Still blocked after `mm`: push 10 mm, retract again.
- `verify`: the at-extruder check. Push until `tn_adv` (at most `mm`), pull
  back until it clears, 2 mm more, then push again: it must trigger within
  `verify_mm` (8). Otherwise back off and start over.

Every job takes `tries` (2), `speed`, `accel`, `hold` and `seq` (echoed in
`job_done`). The driver is
turned off when the job ends unless `hold` is true. Retries send `job` with
`state:"recovering"`; only the end result is a fault. `job_done` `tries` is
how many tries the job took.

## Faults

| code | meaning | lane stops |
|---|---|---|
| `no_hub`, `no_load`, `no_trigger` | a feed job's switch never triggered | yes |
| `not_at_extruder` | the verify job found no filament at the gears | yes |
| `snag` | a feed's switch triggered far too early on every try | yes |
| `hub_not_clear`, `not_clear` | an unload could not clear the switch | yes |
| `buffer_stuck` | the TurtleNeck did not relax when pulled back | yes |
| `runout` | the prep switch opened during a feed or assist | yes |
| `clog` | assist: TurtleNeck full while the extruder moved `assist_fault_mm` | yes |
| `not_feeding` | assist: TurtleNeck pulled tight while the extruder moved `assist_fault_mm` | yes |
| `tmc_uart` | the driver does not answer (no motor power?), or answers but drops writes | yes |
| `tmc_reset` | the driver lost power; it is set up again | yes |
| `tmc_ot`, `tmc_short`, `tmc_uv` | driver over temperature, short, charge pump | yes |
| `tmc_otpw`, `tmc_open` | over temperature warning, open coil while moving | no |
| `tmc_ok` | the driver is healthy again | |
| `link_lost` | no line from the host for `link_timeout_ms` while anything ran | all |
| `move_refused`, `cancelled`, `fault` | a job could not move, was stopped, or hit a driver fault | |
| `link_lost` (job) | a job stopped on link loss | |

## Link loss

While any motor, espooler, job or assist runs, the host must send a line at
least every `link_timeout_ms` (2000). Otherwise everything stops and
`link_lost` is sent once; moves end with `end:"link_lost"` and jobs with
`code:"link_lost"`, and the drivers that held keep holding. Lines sent
while the USB link is down can be lost, so after a reconnect the host asks
`replay` for the result of a move or job it was waiting on, matched by its
`seq`. `{"cmd":"cfg","link_timeout_ms":0}` turns this off for bench tests.

## Boot guard

A build that cannot talk to the host puts itself back in the bootloader, so a
board in a closed case never needs its BOOT or RESET buttons after a bad
flash.

- A power-on boot trusts the build.
- The first boot of a new build, and any boot after a watchdog reset, is a
  trial (`info` `trial:true`): the host must send a line with a known `cmd`
  within 60 s. If not, the board resets and counts a failure.
- After 3 failed trials in a row (`boot_fails`), it restarts into Katapult
  and stays there until it is flashed again or powered off.

AFC answers this by itself, since it sends `info` on every connect. After a
bench flash, send any request within a minute.

## Settings (`cfg`)

| key | default | AFC option it comes from |
|---|---|---|
| `link_timeout_ms` | 2000 | |
| `assist_high`, `assist_low` | 1.05, 0.95 | `multiplier_high`, `multiplier_low` |
| `assist_fault_mm` | 0 | `filament_error_sensitivity` ((11 - s) * 10 mm; 0 = off) |
| `assist_fallback_speed` | 30 | assist speed with no extruder updates |
| `assist_max_speed` | 80 | |
| `espool_fwd`, `espool_rwd` | 0.5, 0.5 | `fwd_speed_multiplier`, `rwd_speed_multiplier` |
| `espool_kick_ms` | 70 | `kick_start_time` |
| `espool_brake_ms` | 200 | `n20_break_delay_time` |
| `espool_cycle_us` | 100000 | `cycle_time` (taken before the first espooler use; later changes need a reboot) |
| `espool_pulse_mm`, `espool_pulse_ms` | 0, 0 | print assist: `delta_movement` and the cruise time AFC works out from the spool weight |
| `espool_follow` | true | espoolers follow lane moves when not set by hand |
| `job_tries` | 2 | |
| `job_backoff_mm`, `job_snag_backoff_mm`, `job_repush_mm` | 20, 30, 10 | |
| `verify_mm`, `verify_relax_mm`, `verify_settle_mm` | 8, 30, 2 | |

The firmware's defaults match a stock BoxTurtle in AFC
(`templates/AFC_Turtle_1.cfg` and the installer's TurtleNeck buffer) so it
boots safely; AFC sends its own values every time it connects.

## Espoolers

While the host has not set an espooler by hand (`espool`), it follows its
lane: forward while the lane feeds, rewind while it retracts, brake and off
when it stops. The duty comes from the move or job line (`espool_fwd`,
`espool_rwd`, 0..1, and `espool_kick`), so AFC can size it from the move
speed and the spool weight with its own `calculate_pwm_value`, and send 0
for the short moves it runs without the espooler. A key the line leaves
out uses the `cfg` value.

While assist runs in `tn` or `ext` mode, the espooler only gives print
assist pulses: full duty (with the kick) for `espool_pulse_ms` every
`espool_pulse_mm` the lane feeds. AFC sends both per lane in `lane_cfg`, from the spool weight
(`calculate_cruise_time` plus `kick_start_time`), 0 ms when the spool is at
or over `enable_assist_weight`, and sends them again as the weight drops.
