# Bambu AMS hot-swap and pooled lanes

A Bambu AMS or AMS HT can be plugged in while Klipper runs. Its lanes, its
`T#` commands and its place in the panel and dryer appear live, with no restart.
Once it has stayed online for `enroll_grace`, its bay is saved, and a restart
brings it back on the same lanes and `T#`. With `auto_drop: True` an unplugged
unit's lanes and `T#` drop live as well. With the default `auto_drop: False`
they stay registered until the next restart, or until the unit is forgotten,
unassigned or moved.

This works by claiming lane objects that already exist rather than creating
new ones. `AFC_BridgeBox` (the chain master) builds every bay up front at
startup, and a plug or unplug only moves a bay between *pooled* and *claimed*.

## Why a pool

Klipper builds its objects when it parses the config. A lane and its status
entry are config-time objects, and there is no supported way to add one to a
running Klipper. `T#` commands are registered at runtime, so a claim can add and
remove those live. Bambu lanes are bus-fed and pinless: the AMS drives its own
motors and the bridge talks to it over the serial bus, so a lane needs no MCU
pins. That is what lets a bay be built ahead of time and handed to whichever
unit turns up.

## The pool

At startup the master builds:

- four-lane AMS bays: one for each recorded AMS, plus spares up to
  `pool_ams`, and never more than four in all;
- one-lane HT bays: one for each recorded HT, plus spares up to `pool_ht`.

The defaults, 4 and 8, match the most a Bambu bus takes. `pool_ams` and
`pool_ht` count total bays with recorded units included, so the number of
spare bays is the pool size minus the units already recorded.

Each bay is an `AFC_BambuAMS` unit carrying a `pool` flag, and each of its
lanes is flagged `unassigned`. The unit is in AFC's unit list, but PREP and the
dryer panel skip units with the `pool` flag. The lanes join no registry and get
no `T#` until a unit claims the bay.

Every bay has a real name from the start, whether or not a unit owns it (see
[Names](#names)). A bay built for a recorded unit carries that unit's UID and
a `temperature_sensor <name>`, even while the unit is unplugged. Spare bays
have no UID and no sensor, so a unit claimed live onto a spare gets its
temperature card at the first restart that builds it a bay of its own, which
needs it to be recorded (and listed in `roster:` while that option is set).

Every bay starts inert, recorded ones included. On a chain with a pool, the
watch tick claims a unit once it reads online. At startup, claims wait for
AFC's PREP to finish (see [Claim and release](#claim-and-release)). A recorded
unit that is powered off at startup shows no lanes or `T#` until it comes
online, and then it claims live.

### Known units: the roster

A unit is *known* when its UID is in the roster. The roster is the `roster:`
option (a `model:uid` list) when that is set, and otherwise the roster recorded
in the state file.

- A unit seen on the chain is written into the recorded roster once it has
  stayed online continuously for `enroll_grace` (default 15 s). The claim does
  not wait for this: its lanes appear at once.
- Once a unit is recorded and has been online for `enroll_grace`, the bay it
  is on is saved for it: its name, lanes and `T#`. The console says:

  `AFC_BridgeBox <chain>: saved <UID> on <bay> (lane24-lane27, T24-T27); it comes back there after a restart.`

  From then on the bay is reserved for it (see [Reserved
  bays](#reserved-bays)), and a restart brings it back on the same bay.
- A unit unplugged before `enroll_grace` is neither recorded nor saved. With
  `auto_drop` its bay goes back to the pool. Without it the unit stays claimed
  and is saved once it has been back online for `enroll_grace`.
- A recorded unit with no saved bay, such as one that waited with no free bay,
  takes the lowest free name of its family at the next restart, when one is
  free. Units with recorded lanes draw before units seen for the first time;
  roster order decides the rest.
- The `roster:` option overrides the recorded roster. With it set, a unit the
  option does not list still claims a spare live, but its bay is not saved,
  `AFC_BRIDGEBOX_ASSIGN` refuses it, and its New AMS popup names the entry to
  add instead of offering bays. Add `<model>:<uid>` to `roster:` and RESTART to
  give it a bay of its own: as a recorded unit with no saved bay, it takes the
  lowest free one of its family, which need not be the one it is on now.
- A `model:uid` entry takes `ams1` (AMS), `ams2` (AMS 2 Pro), `ht`, or `boxed`
  (an AMS whose generation is not confirmed yet). `lite` is accepted but
  reserved for the AMS Lite. Any other tag is a config error. A unit listed in
  `roster:` is claimed as the model written there, except that a `boxed` entry
  takes the `ams1` or `ams2` the bus has already confirmed for that UID.
- While a pool is configured (`pool_ams` or `pool_ht` above 0, which is the
  default) or a `roster:` option is set, the roster is never pruned and
  `removal_grace` has no effect. With a pool, a unit saved on a bay keeps it
  across any number of restarts until it is forgotten with
  `AFC_BRIDGEBOX_FORGET`, unassigned with `AFC_BRIDGEBOX_UNASSIGN`, or moved
  with `AFC_BRIDGEBOX_ASSIGN`.

## Lane layout

Lanes are laid out in fixed bands, so a unit's lanes and `T#` do not move when
other units come and go:

- AMS rank *r* (0 to 3) takes lanes `lane_base + 4r` to `lane_base + 4r + 3`.
- The AMS band is `pool_ams` bays wide, or as wide as the highest recorded AMS
  bay needs, and never more than four bays. It does not shrink under recorded
  HT lanes that sit within four AMS bays: when a lower `pool_ams` would narrow
  it, the HTs stay on their recorded lanes and `T#`. If a hand-written
  section, a chain above, or a chain further down the config whose
  `lane_base` is set or saved already uses one of those HT lanes, the band is
  only as wide as `pool_ams` and the recorded AMS need, the HT lanes move to
  follow it, and the startup log says why, such as
  `[AFC_BridgeBox chain2] further down the config builds lane28`.
- HT rank *r* takes the lane *r* places past the end of the AMS band, so the HT
  band starts at `lane_base + 16` at most.
- Each lane's home tool is `T<lane#>`.

A unit's rank is the position of its name in `ams_names` / `ht_names`, or in
the default names past the end of the list. With no lists set, `Bambu_AMS_1`
is AMS rank 0 and `Bambu_AMS_HT_1` is HT rank 0. How many units are online
never affects the layout, and `pool_ht` moves no lanes.

With `lane_base` 12 and the default pools:

| Bay | Lanes | Tools |
|-----|-------|-------|
| `Bambu_AMS_1` (AMS rank 0) | lane12-15 | `T12`-`T15` |
| `Bambu_AMS_2` (AMS rank 1) | lane16-19 | `T16`-`T19` |
| `Bambu_AMS_3` (AMS rank 2) | lane20-23 | `T20`-`T23` |
| `Bambu_AMS_4` (AMS rank 3) | lane24-27 | `T24`-`T27` |
| `Bambu_AMS_HT_1` (HT rank 0) | lane28 | `T28` |
| `Bambu_AMS_HT_2` (HT rank 1) | lane29 | `T29` |
| ... up to `Bambu_AMS_HT_8` | lane35 | `T35` |

The HT band starts at lane28 whether one AMS is plugged in or four. On a new
install, `pool_ams: 2` starts it at lane20.

`lane_base` defaults to 0, which means automatic: one past the highest `laneN`
in the config's `[AFC_lane]` / `[AFC_stepper]` sections or built by a chain
above this one, else one past the highest `T#` (or `AFC_extruder eN`) in use,
else 24. Once resolved it is locked in the state file, so adding lanes
elsewhere later does not move the Bambu lanes. Setting `lane_base` explicitly
moves every Bambu lane and `T#`, and the moved lanes start without their
saved details (see below).

Set `pool_ams` to the most AMS you will run on this chain. A Bambu bus
addresses at most 4 AMS, so `pool_ams` above 4 is treated as 4 and the startup
log says so. Keep `pool_ht` at 8 or below, the most HT a Bambu bus takes.

- Lowering `pool_ams` moves nothing that is recorded: every recorded AMS keeps
  its bay, and the HT lanes stay where they are. Only the number of spare AMS
  bays drops. While a recorded HT holds the band wider than `pool_ams` and the
  recorded AMS need, the startup log says so at every start and, on a chain
  with a pool, gives the `pool_ams` value that silences it.
- Raising `pool_ams`, or recording an AMS that needs a bay past the AMS band,
  widens the band. When the wider band reaches the HT lanes, every HT's lanes
  and `T#` move up to start past it, 4 lanes for each AMS bay it gains. The
  startup log lists each unit whose lanes moved.
- An HT whose recorded lanes sit past four AMS bays, such as one recorded
  while `pool_ams` was set above 4, moves down at startup, so the HT band
  starts at `lane_base + 16`. Its lanes and `T#` change, and the startup log
  lists each one. Update macros and Spoolman bindings keyed to its old lanes.

Lanes whose numbers change at startup start without their saved details, even
when the unit keeps its name (see [Spools across a
release](#spools-across-a-release)). Settle `pool_ams`, `lane_base` and the
names lists early, before those details matter.

A new unit is claimed only onto a free bay of its family (see [No free bay:
Replace](#no-free-bay-replace) for when none is free).

### Config edits and saved names

A recorded unit's name is saved with its bay, and its lanes follow from that
name. These edits change which names a family is given, and Klipper still
starts after each of them:

- **Renaming or removing** an `ams_names` / `ht_names` entry a unit wears,
  **adding a names list** over default names units wear, or **changing
  `unit_prefix`**: the saved name is no longer one the family is given, so the
  unit draws a free name at startup. An AMS keeps its lanes when the name for
  that lane position is free; otherwise it takes the lowest free name. An HT
  always takes the lowest free HT name, so its lanes and `T#` can change.
- **An AMS saved past the fourth bay** (such as `Bambu_AMS_5`, or the fifth
  `ams_names` entry) draws a free name inside the four AMS bays. If all four
  are held by other recorded AMS, it waits with no bay (see [No free bay:
  Replace](#no-free-bay-replace)), and the startup note says its saved name
  and lanes are dropped. When a bay frees, it takes that bay, with none of its
  old lane details.
- **Reordering** `ams_names` / `ht_names` entries: units keep their names and
  move to the lanes and `T#` of their new positions.
- **Deleting a UID from the roster** without `AFC_BRIDGEBOX_FORGET`: with a
  pool, its name holds no bay. A spare wears that name, and the next unit saved
  on that bay takes the name over. Without a pool the name stays reserved for
  the unit's return (see [Turning live claiming
  off](#turning-live-claiming-off)). FORGET is the clean way to retire a unit.
- **Two UIDs recorded with the same name** (a hand-edited state file): the
  first in roster order keeps it, and the other draws a free name.
- **A `lane_map` entry no unit could have** (a hand-edited span other than 1
  or 4 lanes, or a negative first lane) is ignored, and the unit draws fresh
  lanes.

At startup the console names each recorded unit whose name or lanes changed,
with its old and new lanes and `T#`. Such a unit keeps its learned bowden
lengths, which follow the unit, and its lanes start without their saved
details (see [Spools across a release](#spools-across-a-release)). A lane
that AFC records as loaded to a toolhead and that this layout gives to another
unit is reported once PREP has run (see [Follower
restore](#follower-restore)).

These still stop Klipper starting:

- A bay name that matches the name of a non-Bambu AFC unit in the config (the
  `unit:` of a hand-written `[AFC_lane]` or `[AFC_stepper]`). The error asks
  you to rename that `ams_names` / `ht_names` entry. When the bay has its
  default name, give it another one with an `ams_names` / `ht_names` entry
  instead, or change `unit_prefix`.
- A `roster:` entry that is not `<model>:<uid>`, has an unknown model or
  repeats a UID, or a `roster:` value with no entries in it (such as a lone
  comma). A blank `roster:` counts as unset. The same checks apply to the
  roster recorded in the state file, and the error then names that file.
- A hand-written section that Klipper loads before the chain's own section
  and that has the same name as one the chain builds, such as an
  `[AFC_lane laneN]` on a Bambu lane. The error says it `already exists in
  the config -- remove one`. The check only sees sections loaded before the
  chain's own, so keep hand-written sections off the Bambu lane numbers and
  bay names wherever they sit in the config.
- Two chains that build the same section, such as a second chain added above
  an existing one (see [Names](#names)), or a chain whose `lane_base` (set in
  its section, or locked in the state file) falls inside another chain's
  lanes. The error reads `[AFC_BridgeBox <chain>] would fabricate
  [<section>], but [AFC_BridgeBox <other>] above it in the config already
  builds it` and says what to change. For a lane it says where this chain's
  `lane_base` comes from and to set `lane_base:` in one chain's section past
  the other chain's last lane; for the buffer, to give one chain its own
  `buffer:` name; otherwise, to give one chain its own `unit_prefix`, or its
  own `ams_names` / `ht_names`.
- Two bays on the same lanes. This last-resort check is for a state file
  edited by hand; the error names both bays and the UIDs whose `name_map` and
  `lane_map` entries to remove from the state file by hand.

If Klipper already refuses to start, undo the edit first, since none of the
commands can run until it starts.

## Names

Each rank of a family has a name: its `ams_names` / `ht_names` entry by
position, and past the end of the list the default name, `<unit_prefix>_N`
for an AMS and `<unit_prefix>_HT_N` for an HT, where N is the rank + 1.
`unit_prefix` defaults to `Bambu_AMS`. Only the first four `ams_names` entries
name bays. A recorded unit's bay has the name saved for it, and a spare bay
takes the lowest name its family does not already hold.

```ini
[AFC_BridgeBox chain1]
ams_names: PLA_Station, PETG_Station, Support, Spare_AMS
ht_names:  Dryer_A, Dryer_B
```

Entry *i* names the bay at rank *i*, lowest lanes first. Past the end of a list
the default names take over, so with the two `ht_names` above the third HT bay
is `Bambu_AMS_HT_3`.

Every bay gets a name of its own:

- A default name that an entry already uses gets a suffix. With
  `ams_names: PLA, PETG, Bambu_AMS_4`, the third AMS bay is `Bambu_AMS_4` and
  the fourth is `Bambu_AMS_4_2`.
- An entry gives way when it repeats an earlier entry of its list, when an
  `ht_names` entry is the default name of an AMS bay that `ams_names` does not
  name, and when an `ams_names` entry is an HT name (an `ht_names` entry or an
  HT default name). Its bay then takes its default name, with a suffix when
  another bay has that.

The startup log names each bay named either way.

A second `[AFC_BridgeBox]` chain needs its own `serial_port` and a different
`unit_prefix`, so the names do not collide. `lane_base` can stay automatic:
each chain starts one past the last lane the chains above it built. The first
chain's units register the `bambu_buffer` pin chip and each later chain's units
register `bambu_buffer_<chain name>`; `buffer_chip_name` on a chain overrides
that. A chain that only scouts (see [Turning live claiming
off](#turning-live-claiming-off)) builds no units, so a hand-written
`[AFC_buffer]` on `bambu_buffer` below such chains belongs to the first chain
that builds units, and that chain's units use `bambu_buffer`. Add a new chain
below the existing ones. A chain that builds units, added above an existing
one, takes that chain's lane numbers and chip name, and the lower chain then
refuses to start.

The model generation is not part of a name. AMS 1, AMS 2 Pro and other boxed
units are all one AMS family, and the model sets the heater flag and dry
ceiling. A new AMS claims as `boxed`, with no heater, unless `roster:` lists it
as `ams1` or `ams2`. Once it is recorded and the bus confirms `ams1` or `ams2`,
its heater and dry ceiling follow the confirmed model, including a `heater` or
`dry_max_temp` set in its model section (such as `[AFC_BridgeBox ams2]`), and a
`measure_on_insert` set there is applied live too. Bowden lengths set in the
model section apply from the unit's next claim. Other keys in a model section
take effect at the next restart.

What stays with a bay and what follows the unit:

- The bay's name, lanes and home `T#` stay with the bay, and so do macros
  keyed to them.
- A lane's saved details come back only to the unit that was on the bay (see
  [Spools across a release](#spools-across-a-release)).
- Learned bowden lengths follow the unit's UID to whatever bay it claims (see
  [Learned values](#learned-values)).

`AFC_BRIDGEBOX_FORGET` and `AFC_BRIDGEBOX_UNASSIGN` both release a unit's name.
No command renames a unit and keeps its lanes: `AFC_BRIDGEBOX_ASSIGN` moves a
unit to a different bay, with that bay's lanes, `T#` and name.

### Learned values

The `afc_bowden_length` and `afc_unload_bowden_length` a unit measures are
saved per chain and per unit UID, in the state file under
`[AFC_BridgeBox <chain> learned <UID>]`. They follow the unit to any bay, and
another unit claiming the same bay never inherits them.

At every claim, each length comes from the first of:

- an operator value in `[AFC_BridgeBox <bay name>]` or
  `[AFC_BridgeBox <model>]`, or an `afc_unload_bowden_length` on the chain
  section (`afc_bowden_length` on the chain section sets the hubs, not the
  unit);
- the unit's own saved value;
- the unit's built-in default.

The unload length follows the load length unless it is set on its own. The
console notes when a claimed unit takes its saved lengths. After a claim, a
unit adopts a measured path only from its own completed load. Only a length
that is a positive, finite number is saved or applied.

A bowden length stored under a bay name is handled at startup:

- In the state file, it goes to the unit that wore that name, as far as the
  chain can tell: normally the unit recorded with it. With a `roster:` option,
  a value under the name of a spare that no unit wore or wears goes to the one
  unit of that family that the recorded roster lists and `roster:` does not,
  when there is exactly one such unit and one such spare with values. When the
  chain cannot tell, the value is dropped or left in place. The startup log
  says where each value went, or why it did not.
- In `AFC_auto_vars.cfg`, under the name of a bay built at this startup for a
  recorded unit, it goes to that unit and replaces the unit's saved value.
  Under a spare bay's name, or a name no bay has at this startup, it is
  deleted.

Values AFC saved in `AFC_auto_vars.cfg` for a lane, hub or temperature card
that the chain no longer builds move into the chain's state file, and the
startup log says `auto_vars [<section>] moved to <state file>: no chain builds
that section now`.

## Claim and release

The watch tick runs every `hotplug_poll` (default 1.0 s, minimum 0.5 s), or
every 3 s while the bridge is still coming up. It reads each unit's online
flag from the bridge. The flag goes false about 1.5 s after a unit stops
answering, so an unplug is noticed about 1.5 to 2.5 s after it happens.

At startup, claims wait for AFC's PREP to finish, so each lane gets only its
own unit's saved details. The wait lasts at most 90 s after Klipper is ready,
or `moonraker_timeout` + 60 s when that is longer; past that the console warns
and units are claimed anyway. A buffer of type `bambu` counts its
`odom_bind_grace_seconds` wait for a unit's odometer from its first reading
after PREP, so this wait does not leave it judging loads on buffer pressure
alone.

**Claim.** A UID that reads online and is not on a bay is claimed once it has
stayed online for `claim_grace` (default 0, so at once). It takes the first
of:

1. the bay reserved for it;
2. a free bay of its family wearing the name it is saved under;
3. the free bay it was last claimed onto, unless another recorded or waiting
   unit's saved name is on that bay;
4. the lowest free bay of its family that no other recorded or waiting unit's
   saved name is on (a waiting unit is one online and not yet claimed);
5. the lowest free bay of its family.

An HT only lands on an HT bay and an AMS only on an AMS bay. Within one tick,
units coming back to a bay claim first, lowest lanes first, and then new units
in UID order, so a returning unit and a new one plugged in together do not
swap bays. If no bay of its family is free, the unit waits (see [No free bay:
Replace](#no-free-bay-replace)).

Claiming registers the bay's lanes, puts the unit's saved lane details on
them (see [Spools across a release](#spools-across-a-release)), gives each
lane its `T#` (see [T# commands](#t-commands)), saves them, applies the
heater, dry ceiling and learned values for the unit and its model, and brings
the unit live.

**Release.** Release needs `auto_drop: True` and no print running. A claimed
unit that has been offline for `release_grace` (default 10 s, minimum 2) is
dropped: its lanes are unregistered and its `T#` commands removed. Other units
are not touched.

- A unit with a lane AFC records as loaded to a toolhead stays claimed. The
  console says so once per absence and suggests unloading it or, if the
  filament is already out, `UNSET_LANE_LOADED`, which clears the record only
  while that lane's toolhead is the active one; the line names that tool when
  it is not. The unit is released on the next check after that.
- An online reading cancels a pending drop only once the unit has stayed online
  continuously for `release_settle` (default 5 s). A pulled unit whose online
  flag blips on for a moment still drops.
- The drop fires only on a tick where the unit reads offline, so a unit that is
  back and reading solidly online is never dropped.
- A bay saved for the unit stays reserved for it. Any other bay goes back to
  the pool.

**Anti-flap.** A unit that `auto_drop` released within the last `flap_window`
(default 120 s) must stay online for `flap_claim_grace` (default 15 s) before
it can claim again, so a flapping cable cannot thrash the pool. A release by a
command does not start this wait.

### T# commands

A claimed lane gets its saved `T#` map when the same unit comes back to the
same bay, and its home `T<lane#>` otherwise. `SET_MAP`, `AFC_SWAP_MAPPING`,
`AFC_ADD_MAPPING` and `AFC_REMOVE_MAPPING` on Bambu lanes are saved like any
AFC lane's map. The map comes back when the same unit returns to the same bay,
across a restart or an `auto_drop` re-plug, and it is lost with the rest of
the lane's details (see [Spools across a
release](#spools-across-a-release)).

- A lane remapped away from its home `T#` does not take it back at a claim;
  the lane that received it keeps it.
- A lane saved with no `T#` (its last `T#` removed with `AFC_REMOVE_MAPPING`,
  or taken by another lane's `SET_MAP` with multiple mapping on) comes back
  with none. `AFC_ADD_MAPPING` gives it one again, or `SET_MAP` takes one from
  another lane.
- A saved `T#` that another lane or a macro holds at the claim stays where it
  is and is dropped from the map, with a warning. A lane left with nothing
  goes back to its home `T#` while that is free, and to no `T#` when it is not,
  with a warning to use `SET_MAP`. A lane left with no `T#` this way stays
  without one at later claims until you give it one.
- When a lane takes its home `T#` from another lane, a warning names both:
  - a claimed Bambu lane that holds it as its only `T#` goes back to its own
    home `T#` when that is free;
  - any other lane, such as a lane PREP numbered into the Bambu range, loses
    that `T#`. A lane left with no other `T#` gets the lowest free one past the
    Bambu lanes, which works at once and is saved; the warning says how to
    pick another tool with `SET_MAP`. If the lane's config section sets `map:`
    to the taken `T#`, the warning also says to change it there, or
    `AFC_RESET_MAPPING` puts the lane back on it.
- The home `T#` wins between Bambu lanes with no warning: when both lanes are
  Bambu lanes and each keeps its own home `T#`, the dropped or moved `T#` goes
  to AFC.log only.
- During a print a claim takes no `T#` that another lane or a macro holds
  (see [The print gate](#the-print-gate)).
- The `CLAIMED` console line lists each lane restored onto a saved map other
  than its home `T#`, such as `Saved maps: lane25->NONE.` for a lane saved
  with none.

`AFC_RESET_MAPPING`, and `AFC_ENABLE_MULTIPLE_MAPPING ENABLE=0`, which runs
it, put every claimed Bambu lane back on `T<lane#>` and number the other lanes
from `T0` as usual. The result is saved. Two cases leave a Bambu lane to AFC's
usual numbering, with a console warning once per Klipper session and AFC.log
lines after that:

- Another lane's config section has `map: T<lane#>` as its first `T#`. Set
  that `map:` outside the Bambu lanes.
- `T<lane#>` is a macro other than AFC's tool change, which AFC does not
  replace. Remove or rename that macro.

A reset changes only claimed lanes. A bay held for an unplugged unit keeps its
saved map, which comes back with the unit; a later reset puts it home.

`SET_MAP LANE=<lane> MAP=T<n>` swaps `T#`s with the lane that holds `T<n>`,
and refuses a `T#` no lane holds. With multiple mapping on, it moves the `T#`
instead, and the lane that held it is left without it.

Keep other lanes' maps and your own `T#` macros outside the Bambu lanes: a
Bambu lane cannot have a `T<lane#>` that one of them holds.

### Reserved bays

A bay is reserved for a unit that is saved on it (see [Known
units](#known-units-the-roster)). While the unit is unplugged, no other unit
claims that bay, and a re-plug takes back the same lanes, name and `T#` with no
popup.

A unit that is not saved on its bay holds it only while it is claimed. When
`auto_drop` releases it, the bay goes back to the free pool. A re-plug takes a
free bay in the order under [Claim and release](#claim-and-release), its last
bay first, and the New AMS popup appears again (see [The print
gate](#the-print-gate) for a claim during a print). `AFC_BRIDGEBOX_ASSIGN`
saves a unit's bay at once, so a bay it assigns stays reserved across an
`auto_drop`.

A recorded unit is not saved, and the console warns once, when it is on:

- a bay of the other family (an HT bay has one lane, an AMS bay four).
  `AFC_BRIDGEBOX_UNASSIGN UID=<uid> FORCE=1` re-homes it;
- a bay whose name is saved for another recorded unit, which gets that bay at
  the next restart.

The save happens on the next check after the cause is cleared. For a chain
without a pool, see [Turning live claiming off](#turning-live-claiming-off).

### Spools across a release

There are two different UIDs. The *unit UID* is the AMS unit's hardware ID. It
decides which bay the unit claims and whose saved lane details it gets. The
*spool UID* is the RFID tag on each reel and is what Spoolman matches.

A lane's details (Spoolman spool, material, colour, weight, temperatures,
variant, runout lane, TD-1 data and `T#` map) are saved for the unit on the
bay, under the bay's name and lane names. They come back only when the same
unit UID claims the same bay again, with the same name and lane numbers:

- On release the lanes are cleared of all of the spool's details, and their
  tare, density and diameter go back to the lane's config. The details are
  held for that unit's next claim of the bay.
- AFC's variable file keeps them across a restart. While no unit is claimed
  on a bay, every save of that file writes the details held for the bay's
  unit, so they survive a restart with the unit unplugged all session,
  released, or not yet claimed after PREP.

A claim puts the saved details on the lanes at once and saves them. About 8 s
later, or later if the bridge has not polled the unit yet, each bay is checked
against its lane:

- An empty bay's lane is cleared.
- An untagged spool keeps the saved details, and one console line per unit
  names those lanes, such as
  `restored the saved records of the untagged spools on lane13 (spool 12), lane14`.
  An untagged spool swapped while the printer was off keeps the old spool's
  details too, since the swap cannot be seen.
- A tagged spool keeps the saved details unless its tag shows another spool:
  its Spoolman link, weight, variant and temperatures stay, and the lane takes
  the tag's material and colour, so a material or colour set by hand gives way
  to the tag's.
  - With a Spoolman link, only Spoolman can show another spool, as the saved
    details carry no tag UID. A tag with a reel (tray) UID shows another
    spool when the linked spool records a different reel
    (`Spoolman records another reel for spool N`). A tag with no reel UID
    shows another spool when the linked spool records other tags and not
    this one (`Spoolman records other tags for spool N`). A linked spool
    that records neither, as one linked by hand does, keeps its link whatever
    the tag says, and so does any link while Spoolman cannot be reached or
    without `[AFC_BambuAMS_rfid]`. A spool swapped for another, even of
    another material, then stays on the old link until you relink the lane.
  - Without a link, the tag shows another spool when the variant or colour
    differs where both have one, with a black tag read as black. A different
    material counts only when the colours cannot be compared.

  The tag then replaces the details and their link, and the console says
  `the tag replaces the saved record`. After a link is dropped this way, the
  bay's Spoolman lookups only match existing spools and never create one,
  until the bay is emptied or scanned. A tagged spool with no saved details
  goes by its tag, and Spoolman is matched by the tag's UID.
- A bay whose tag has not been reported yet keeps the saved details until the
  tag arrives, and the tag is then checked the same way.
- Details that are only AFC's defaults (no Spoolman link or variant, AFC's
  default material and colour, no extruder temperature, and a weight of 0 or
  1000 g) give way to any tag. On an untagged spool they stay, and the console
  says
  `lane16 has only the AFC defaults saved last session, as nothing has read its spool; reseat it, or run AFC_BAMBU_SCAN LANE=lane16`.
  A weight AFC has counted down, or a temperature set by hand, is more than
  the defaults.

An untagged spool with no saved details for it (a new unit, a different unit
on the bay, or a unit whose name or lanes changed) gets AFC's defaults. At a
bay's first claim after Klipper starts it is also scanned once as a fresh
insert, which moves filament. The console says the slot
`came up with a spool and no record of it -- scanning it as a fresh insert`.
The usual insert rules apply, so nothing is scanned during a print.

The saved details are gone, and the lanes start clean (home `T#`, then tags or
AFC defaults), when:

- a different unit claims the bay;
- the unit claims another bay, such as one it is moved to with
  `AFC_BRIDGEBOX_ASSIGN`, or a free one after a release or
  `AFC_BRIDGEBOX_UNASSIGN`. Claiming the first bay again does not bring them
  back;
- the unit's name or lanes change at startup: it drew a new name, it was left
  with no bay, a names list was reordered, `lane_base` was set, or the HT
  lanes moved (up as the AMS band widens, or down from past four AMS bays;
  see [Lane layout](#lane-layout));
- the unit is forgotten, directly or by `AFC_BRIDGEBOX_REPLACE`.

On the first start from a state file with no bay owners in it, as written by
a build that does not record them, a bay's details go to the unit reserved on
it when that unit's name was saved for the bay, or when the unit had no saved
name and no unit had the bay's name. With a `roster:` option, the details of a
spare whose name no unit had go to the one unit of its family that the
recorded roster lists and `roster:` does not, when that is the only such unit
and the only such spare with details. Every other bay starts clean. These
guesses are saved as the bays' owners, so later starts do not guess again.

## No free bay: Replace

On a chain with a pool, a unit that finds no free bay of its family is not
claimed. It waits on the chain with no lanes and claims the first bay of its
family that frees, live: for example after `AFC_BRIDGEBOX_FORGET` of the unit
on it, or `AFC_BRIDGEBOX_UNASSIGN` of that unit while it is offline. A unit
unassigned while online (`FORCE=1`) claims its own bay straight back. A bay
held for a known unit stays taken while that unit is unplugged (see [Reserved
bays](#reserved-bays)), so a unit that replaces a known one waits until the
old one is forgotten, or until Replace (below) gives it that bay.

The console says so once per wait, and again only after the unit drops off a
chain that is still answering and comes back. The line gives the command that
frees a bay. For an AMS facing four held AMS bays it names every unit holding
one and marks the offline ones; otherwise, on a chain with a pool and no
`roster:` option, it names the offline units holding a bay of its family:

- **An AMS when four AMS bays are held,** or when the roster a restart builds
  from already gives four other AMS a bay. A Bambu bus addresses at most 4
  AMS, so the AMS band never grows past four bays, and neither a higher
  `pool_ams` nor a restart gives it one. It is pointed at the unit it
  replaces: `AFC_BRIDGEBOX_REPLACE`, or `AFC_BRIDGEBOX_FORGET` of that unit.
  With a `roster:` option set, the line also says which `roster:` entry to
  change. Without a pool, the unit takes the bay the FORGET frees at the next
  restart, or with `AFC_BRIDGEBOX_ASSIGN` while it is online.
- **An HT, or an AMS while fewer than four AMS bays are built.** Once the unit
  is recorded, a restart builds it a bay. An AMS bay past the AMS band moves
  every HT's lanes and `T#` up by 4, and their saved lane details do not follow
  (see [Lane layout](#lane-layout)). Raise `pool_ams` or `pool_ht` to keep
  spare bays for units plugged in live. An offline holder is named with the
  same FORGET and REPLACE commands.
- **A unit a `roster:` option does not list,** while the option leaves it
  room: add its entry to `roster:` and RESTART to give it a bay.

**Replace.** On a chain with a pool and no `roster:` option, a waiting unit can
take the bay of a unit that is offline. `AFC_BRIDGEBOX_REPLACE` forgets the old
unit and assigns the new one to its bay in one step. The new unit takes the
bay's name, lanes and home `T#`, saved across restarts. It gets none of the old
unit's lane details, spools, `T#` maps or learned values, which the forget
erases.

The **No free bay for new AMS** popup (**No free bay for new AMS HT** for an
HT) offers this. For a recorded AMS that this start gave no bay it is titled
**No bay for AMS `<uid>`**, and says so, adding that a Bambu bus addresses at
most 4 AMS when its saved bay lies past the four AMS bays. It is offered when
all of these hold:

- the waiting unit has been online continuously for `enroll_grace`;
- a bay of its family is held for a unit that has been offline for
  `release_grace` or longer (an *offline-held bay*). Absence counts only while
  the bridge link is up and some unit on the chain is online, and a unit that
  comes back keeps its clock until it has stayed online for `release_settle`;
- at least one such bay has no lane that AFC records as loaded to the toolhead.

It is offered once per wait, and again after the unit is unplugged and plugged
back in. It never appears with a `roster:` option or without a pool; for a
print, see [The print gate](#the-print-gate).

The popup lists each offline-held bay: its name, the old UID, its lanes and
`T#`, and how long the unit has been offline, with a **Replace `<bay>`** button
(up to 8). A bay with a lane in the toolhead has no button, and its row says
how to clear the record (see [Commands](#commands)); during a print it says to
replace it once the print ends instead, with no `FORCE=1`, which would also
override the print gate. Dismiss leaves the unit waiting;
`AFC_BRIDGEBOX_REPLACE UID=<uid>` opens the popup again. The bay manager
(`AFC_BRIDGEBOX_BAYS`) also offers Replace (see [Commands](#commands)).

## The print gate

A print counts as active when `print_stats` is `printing` or `paused`, or
`idle_timeout` is `Printing`. Klipper reports `Printing` there whenever it is
busy running G-code, so a long macro, homing or a heat-and-wait counts too.

- **Release is gated.** A unit pulled during a print stays claimed, with its
  lanes and follower untouched. Its release clock keeps running, so if it is
  still offline when the print ends and `release_grace` has passed, it drops on
  the next tick.
- **Claim is not gated.** A unit plugged in during a print is claimed at once,
  with its saved details, and adding lanes and `T#` does not disturb the
  print. The claim takes no `T#` that another lane or a macro holds, since the
  print may be using it. The lane keeps the rest of its map, or has no `T#`,
  and the console says once:

  `AFC_BridgeBox <chain>: lane16 takes T16 from lane28 once the print ends, as the print may be using it; until then lane16 has no T#.`

  When only `idle_timeout` reports `Printing`, it says `once the printer is
  idle, as a running command may be using it`. Once the printer is idle the
  lane takes those `T#` as at any claim (see [T# commands](#t-commands)), and
  the console says
  `the printer is idle, so the T#s the claim left in use are taken: lane16 is T16.`
  A restart before then keeps that plan. The bridge saves its record of the
  new unit when the print ends rather than mid-print.
- **Popups.** The New AMS popup is not shown for a unit claimed during a print;
  open the bay picker afterwards with `AFC_BRIDGEBOX_ASSIGN UID=<uid>`. The No
  free bay popup waits for the print to end.
- **Commands.** `AFC_BRIDGEBOX_FORGET` of a unit claimed on a bay is refused
  during a print unless `FORCE=1`, and so is `AFC_BRIDGEBOX_REPLACE`.
  `AFC_BRIDGEBOX_ASSIGN`, `AFC_BRIDGEBOX_UNASSIGN FORCE=1` and FORGET of a
  unit that is not claimed, such as one waiting with no bay, are not gated:
  moving or unassigning a live unit mid-print drops its lanes at once. All of
  them still refuse a bay with a lane AFC records as loaded to the toolhead
  unless `FORCE=1` (see [Commands](#commands)); during a print that refusal
  says `FORCE=1` also pulls the lane out from under the print.

## Follower restore

Each claim schedules a follower restore about 8 s later. That covers every unit
at startup, and a re-plug after `auto_drop` released the unit.

A unit that stays claimed while it is away (`auto_drop` off, back online before
`release_grace` runs out, pulled during a print, or kept claimed for a lane in
the toolhead) gets the same restore when it comes back: once it has read
offline for about 2 s or more and then stayed online for `release_settle`.
This runs during a print too. A single offline blip does nothing.

Only the unit owning the lane AFC's extruder records as loaded does anything.
It tells the AMS the filament is at the extruder (the load-complete handoff),
selects that lane so its tray follows the extruder, and turns assist on, and
the console names the lane. It engages once per claim or return, and the host
runs no keep-alive timer. If the unit's chain index is not resolved yet, it
retries every 3 s, up to 6 times. On a return it stands aside while a load or
unload owns the follower, a stall hold keeps it off, or
`AFC_BAMBU_FOLLOWER ENABLE=0` has turned it off.

The claim marks that lane as loaded to the toolhead again, as PREP does at
startup, only when the lane was saved loaded under the claiming unit.
Otherwise the console warns that the filament in that extruder is not from
this unit, the lane is left unloaded with no follower, and the line says to
unload that filament by hand and run `UNSET_LANE_LOADED`.

A lane AFC records as loaded that this startup's layout gives to a different
unit or to no bay (a unit drew a new name, the HT lanes moved, or an AMS was
left with no bay) is not followed by the new unit. Once PREP has run, the
console names the extruder, the lane and both units. Take that filament out by
hand, then clear the record as the console line says. `UNSET_LANE_LOADED`
clears it only while a unit is claimed on the lane's new bay and that lane's
toolhead is the active one; for a lane on no bay, or on a bay no unit has
claimed, it does nothing.

Separately, after a bridge reconnect, units re-assert the loaded lane's
follower about 2 s later.

## Popups

Popups use Klipper's `action:prompt`, so Mainsail and Fluidd show them with no
panel changes.

- **New AMS on `<bay>`** appears when a unit is claimed onto a bay that is not
  its own saved bay. It is already live there. The text says whether the bay is
  saved for it, or will be once it has been online for `enroll_grace`. The
  buttons offer up to 8 other free bays of the same family, lowest lanes first;
  Dismiss keeps it where it is. For a unit a `roster:` option does not list,
  there are no buttons, only the `roster:` entry to add.
- **AMS removed: `<bay>`** appears only when `auto_drop` releases a unit. It
  says whether the bay is held for a re-plug, or went back to the pool, in which
  case a re-plug takes a free bay of its family, its last one first (see
  [Claim and release](#claim-and-release)). Its Forget button runs
  `AFC_BRIDGEBOX_FORGET` for that unit.
- **No free bay for new AMS** / **No free bay for new AMS HT** (or **No bay
  for AMS `<uid>`**) offers the bay of an offline unit to a unit waiting with
  no bay (see [No free bay: Replace](#no-free-bay-replace)).

Several events at once are queued and shown one at a time; a repeat of the
same popup for the same unit replaces the one already queued. Automatic popups
close after 20 s, and the next queued one follows. Popups opened by command
stay up for 180 s. Clicking a button runs the command, and the dialog closes
once the command has run. If the command is refused, the dialog stays up until
it times out and the console says why. The bay manager's Replace button opens
the replace popup in its place.

## Commands

All of these take `CHAIN=<name>`. The first `[AFC_BridgeBox]` also answers
without it; a second chain must be named.

**A lane in the toolhead.** FORGET, UNASSIGN, ASSIGN (moving a unit off its
bay) and REPLACE refuse a bay with a lane AFC records as loaded to the
toolhead unless `FORCE=1`, which clears that record, and the console names
each lane it cleared. In FORGET and UNASSIGN this check comes first, so the
refusal names the lane. On a claimed bay, unload the lane, or, if the filament
is already out, run `UNSET_LANE_LOADED` with that lane's tool active. On a bay
no unit is claimed on, the lanes are not registered, so no unload or
`UNSET_LANE_LOADED` reaches them: plug the unit back in and unload it, or take
the filament out by hand and use `FORCE=1`. Before PREP has run they refuse
any bay no unit is claimed on, since which lane AFC records as loaded is not
known until then.

- `AFC_BRIDGEBOX_ASSIGN UID=<uid> NAME=<bay> [FORCE=1]` pins that unit to that
  bay and saves the pin, its name and a roster entry at once. If the unit is
  online it moves there live. With a pool, a unit that is offline claims that
  bay when it next comes online, and one assigned before PREP has finished at
  startup claims it once PREP has run. Without a pool, it brings the unit's
  lanes up only when run while the unit is online and PREP has finished, and
  for an offline unit the reply says to run it again then, and after every
  restart (see [Turning live claiming off](#turning-live-claiming-off)). Its
  learned values go with it, and its saved lane details do not.
  - It refuses a bay another UID is on or is saved for, and a UID a `roster:`
    option does not list: add it to `roster:`, RESTART, then assign it.
  - It refuses a bay of the other family for any unit the chain knows:
    recorded, on a bay, or waiting for one. For any other UID the family is
    not checked and the unit is recorded with the bay's model, so check it
    yourself (the New AMS popup only offers same-family bays).
  - Moving a unit off a bay with a lane in the toolhead is refused unless
    `FORCE=1` (see above).
  - It lifts a FORGET hold (below).
- `AFC_BRIDGEBOX_ASSIGN UID=<uid>` with no `NAME` opens the bay picker for a
  unit that is on a bay, claimed or reserved. It is the same dialog as the New
  AMS popup, and it offers only the other free bays, never the bay the unit is
  on. For a reserved unit that is not claimed, it says the unit's lanes and
  `T#` are not live yet. For a unit waiting with no bay it says every bay of
  its family is taken, and, without a `roster:` option, names
  `AFC_BRIDGEBOX_REPLACE` while a bay of its family is held for an offline
  unit. For a unit that is neither on a bay nor waiting for one, it is
  refused: plug the unit in first, or give `NAME`.
- `AFC_BRIDGEBOX_UNASSIGN UID=<uid>` (or `NAME=<bay>`) `[FORCE=1]` removes the
  unit's bay and name pin and keeps its roster entry, its learned values and
  its saved lane details, which come back if it claims the same bay again.
  With a pool it takes a free bay of its family live, its last one first, and
  is saved there once it has been online for `enroll_grace`; without one it
  takes the lowest free bay of its family at the next restart. Use
  `AFC_BRIDGEBOX_ASSIGN` to choose its bay. It refuses a bay with a lane in the
  toolhead (see above), then a unit that is online unless `FORCE=1`, which
  drops its lanes first.
  - A unit still plugged in does not give up its bay this way. On the next
    check it claims the same bay back, as its last one, and since it has
    already been online for `enroll_grace` it is saved there again at once;
    the New AMS popup then offers the other free bays. To free the bay for
    another unit, unplug the unit first, move it with `AFC_BRIDGEBOX_ASSIGN`,
    or forget it.
- `AFC_BRIDGEBOX_BAYS` opens the bay manager: every bay, its family and its
  occupant, with an Unassign button (`FORCE=1`) for occupied bays, lowest lanes
  first, up to 8 buttons in all. The button drops a live unit's lanes at once,
  even during a print, and a unit still plugged in comes straight back to the
  same bay (see `AFC_BRIDGEBOX_UNASSIGN` above). A bay with a lane in the
  toolhead has no button and a note to unload it first, or, for a bay no unit
  is claimed on, to plug its unit back in and unload it. Before PREP has run,
  a bay reserved for a unit that is not claimed has no button either. The
  typed command with `FORCE=1` still releases either. Units waiting with no
  bay are listed after the bays. Each gets a Replace button, placed ahead of
  the Unassign buttons, while its family has an offline-held bay (see [No free
  bay: Replace](#no-free-bay-replace)); there is none with a `roster:` option.
- `AFC_BRIDGEBOX_FORGET UID=<uid>` (or `NAME=<unit name>`) `[FORCE=1]` retires
  a unit. It removes the unit from the recorded roster, releases its name and
  lane pin, and erases its learned values, its saved lane details and what is
  stored under its name and lanes in the state file and `auto_vars_file`. A
  name or lanes another UID is also recorded with are not erased or freed. It
  also tells the bridge to forget the unit, and accepts a UID that has only
  learned values left, holds a bay without being recorded yet, or is named
  only as a bay's last owner.
  - If the unit holds a bay this session, the bay is freed live and its lanes
    and `T#` drop. With a pool, the next unit of that family that needs a bay
    claims it with no restart, including one waiting with no bay. Without a
    pool nothing claims it: `AFC_BRIDGEBOX_ASSIGN` another unit onto it while
    that unit is online, or RESTART. For a unit that holds no bay, the roster
    change applies at the next restart.
  - If the unit is still plugged in, its UID is held so the watch tick does not
    re-enroll it at once. The hold clears when the unit is physically pulled,
    so a re-plug enrolls it fresh. `AFC_BRIDGEBOX_ASSIGN` also lifts the hold.
  - A bay with a lane in the toolhead is refused unless `FORCE=1` (see above),
    and so is a unit claimed on a bay during a print (see [The print
    gate](#the-print-gate)).
  - With no arguments it opens a picker listing every recorded unit, with up to
    8 Forget buttons. Units online are marked `LIVE (drops now)`.
  - FORGET does not edit a `roster:` option and warns when the UID is still
    listed there.
- `AFC_BRIDGEBOX_REPLACE UID=<new uid> OLD=<old uid or bay name> [FORCE=1]`
  forgets the old unit and assigns the new one to its bay (see [No free bay:
  Replace](#no-free-bay-replace)). The console line saying which unit replaces
  which is followed by FORGET's and ASSIGN's own lines. A new unit that is
  offline is pinned to the bay and claims it when it next comes online.
  - Without `UID` it uses the one unit waiting for a bay. Without `OLD` it opens
    the replace popup for 180 s, or says why no bay can be offered. That reply
    offers `FORCE=1` only while no print is running and PREP has run, and says
    when it also clears a toolhead record.
  - It refuses, before changing anything and even with `FORCE=1`: a chain with a
    `roster:` option (change the unit's entry there, FORGET the old unit, then
    RESTART) or without a pool; a new UID this chain knows nothing about; a
    new unit that is already on a bay (use ASSIGN to move it); an unknown bay,
    a bay of the other family, a free bay (use ASSIGN), or a bay saved for
    another recorded unit; an old unit that is online, or whose online state
    cannot be read because the bridge link is down or has not reported the
    chain yet.
  - Without `FORCE=1` it also refuses during a print, before PREP has run, when
    AFC records a lane of the bay as loaded to the toolhead, and when the old
    unit has not been offline for `release_grace` on a live chain. `FORCE=1`
    overrides these and clears the toolhead record.

The `AFC_BridgeBox <chain>` status object reports:

- `waiting_for_bay`: units on the chain with no bay;
- `missing`: with a pool, seconds offline for each unit a bay is held for,
  until `release_grace`; without a pool, seconds offline for each absent
  recorded unit, counting toward `removal_grace` (none with a `roster:`
  option);
- `held_bays`: the bays whose saved lane details are held, and for which unit;
- `pending_restart`: what the next restart adds to, removes from or re-models
  in the roster, always empty with a `roster:` option;
- `tombstones`: departed units still recorded with lanes or names, the ones
  `AFC_BRIDGEBOX_FORGET` is for.

## Turning live claiming off

`pool_ams: 0` together with `pool_ht: 0` turns live claim and release off. With
no roster, the master only scouts: units on the chain are recorded in the state
file, and a RESTART builds each of them a bay.

Every bay on a chain without a pool stays inert, whether it comes from a
`roster:` option or the recorded roster. Nothing claims it, so its lanes never
appear unless `AFC_BRIDGEBOX_ASSIGN UID=<uid> NAME=<its bay>` is run while that
unit is online and PREP has finished, and this has to be done again after every
restart. An offline unit is not claimed when it later comes online, nothing is
released until the next restart, and no bay is saved on its own; only
`AFC_BRIDGEBOX_ASSIGN` saves one.

Without a pool or a `roster:` option, a recorded unit offline on a live chain
for `removal_grace` is removed from the recorded roster, which applies at the
next restart. Its name, lanes and learned values stay recorded for its
return. Its name stays reserved, unless a new AMS would otherwise have no bay
or push the HT lanes up; a new AMS takes a free bay inside the AMS band before
a departed unit's name. The console line names any unit waiting for a bay
that takes one this way, and gives the `AFC_BRIDGEBOX_FORGET` command that
frees a departed unit's lanes and name for reuse. The only unit on a chain is
never removed this way, since its absence looks the same as the chain being
off.

## Unclaimed lanes

An unclaimed lane has an empty map and none of a spool's details (no spool ID,
material, colour, weight, temperatures or variant), and is in no unit, AFC,
hub, extruder or buffer lane list. Most of AFC's sweeps pass over it with no
special case.

Two flags move together: `lane.unassigned` gates lane registration and the
release path, and the unit-level `pool` flag is what PREP, the dryer panel and
the `AFC_BambuAMS` status paths check. A claim clears both and a release sets
both.

## Footprint on shared AFC files

The feature lives in `AFC_BridgeBox.py`, `AFC_BambuAMS.py` and the bridge
module. `activate_from_pool`, `deactivate_to_pool`, `assign_pool_tcmd`,
`lane_in_toolhead` and `unset_tool_loaded` are module-level functions in
`AFC_BridgeBox.py`. The shared files carry small edits:

- `AFC_lane.py`: the `unassigned` option and the registration gate.
- `AFC_extruder.py`: `_lanes_pending`, so an extruder whose only lanes are
  pooled is not refused at ready as a standalone extruder with
  `pin_tool_start: buffer`. `AFC_BridgeBox` takes it out of standalone mode
  once a claim gives it lanes.
- `AFC_prep.py`: skips pool units.
- `afc_dryer.py`: keeps pool units out of the live panel.

AFC's own code is not edited for saved lane details: at ready the chain
master wraps AFC's variable-file write queue, so each save writes the details
held for a bay no unit is claimed on.

A re-claim avoids a duplicate `T#` because release unregisters the lane's `T#`
commands, the claim clears stale tool-table entries for the bay's lanes, and
`assign_pool_tcmd` registers nothing when every `T#` on the lane already points
at AFC's `CHANGE_TOOL`.

## Config options

Set on the `[AFC_BridgeBox <name>]` section:

| Option | Default | Effect |
|--------|---------|--------|
| `pool_ams` | `4` | Total four-lane AMS bays, recorded units included. At most 4; a higher value is treated as 4. It is the smallest AMS band width: the band also covers every recorded AMS's bay (never more than four), does not shrink under recorded HT lanes within the four bays, and the HT band starts after it (see [Lane layout](#lane-layout)). |
| `pool_ht` | `8` | Total one-lane HT bays, recorded units included. |
| `ams_names` | empty | Comma list naming the AMS bays, lowest lanes first. Only the first four entries are used. |
| `ht_names` | empty | Comma list naming the HT bays, lowest lanes first. |
| `unit_prefix` | `Bambu_AMS` | Default bay names `<prefix>_N` and `<prefix>_HT_N`. |
| `lane_base` | `0` | First Bambu lane. 0 resolves it automatically and locks it in the state file. |
| `roster` | empty | `model:uid` list of known units, for example `ht:0123456789ABCDEF00003331`. The model is `ams1`, `ams2`, `ht`, `boxed` or `lite`. When set, it overrides the recorded roster. |
| `state_file` | the file holding this section | Where the roster, names, lane map, bay owners, `lane_base` and learned values are kept, as `#~#` lines. Falls back to `~/printer_data/config/AFC/AFC_BridgeBox.cfg`. |
| `auto_vars_file` | `~/printer_data/config/AFC/AFC_auto_vars.cfg` | AFC's file of learned values. The chain reads bowden lengths saved there under a bay name at startup (see [Learned values](#learned-values)), and `AFC_BRIDGEBOX_FORGET` erases what is stored there under the unit's name and lanes. |
| `buffer_chip_name` | `bambu_buffer` on the first chain, `bambu_buffer_<chain name>` on later ones | The pin chip this chain's units register for the buffer (see [Names](#names)). |
| `hotplug_poll` | `1.0` | Seconds between watch ticks (minimum 0.5). |
| `claim_grace` | `0` | Seconds a unit must stay online before it claims. |
| `flap_window` | `120` | Seconds after an `auto_drop` release during which `flap_claim_grace` applies. |
| `flap_claim_grace` | `15` | Seconds a unit `auto_drop` recently released must stay online before it claims again. |
| `enroll_grace` | `15` | Seconds a unit must stay online before it is written into the recorded roster and its bay is saved. Also how long a waiting unit must stay online before the No free bay popup. |
| `auto_drop` | `False` | Drop a unit's lanes and `T#` live once it has been offline for `release_grace`. Needs a pool. Never during a print, and never while AFC records one of its lanes as loaded to the toolhead. |
| `release_grace` | `10` | Seconds offline before a claimed unit is dropped (minimum 2). Also how long a unit must be offline before its bay is offered to a waiting unit. |
| `release_settle` | `5` | Seconds a unit must stay online before an online reading cancels a pending drop. |
| `removal_grace` | `120` | Without a pool: seconds offline on a live chain before a recorded unit is removed from the recorded roster (0 turns this off). No effect while a pool is configured or a `roster:` option is set. |
| `dry_max_temp` | `0` | Caps every heated unit's dry temperature at the lower of this and the model's own ceiling (AMS 2 Pro 65, HT 85). 0 uses each model's own ceiling. A negative value is ignored, with a warning at startup. A `dry_max_temp` in a model or unit section sets that ceiling directly. |
