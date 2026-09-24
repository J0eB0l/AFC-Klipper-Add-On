# AMS narration fixtures

Real narration lines from the three AMS models, one file per scan, with what
the bridge's patterns are expected to make of each line.

The three units narrate the same events in three dialects. Anchoring a pattern
on one of them matches nothing on the other two, and the failure is silent: the
pattern still compiles, still matches its own model, and reads as working code.
That has cost this project two long hunts -- `_RFID_READ_OK_RE` hard-coded
`STEP:` and was HT-blind for months, and none of the `_AMS_HUMAN` rules fired on
an AMS 1 at all. Prose in `docs/` did not prevent either one. These files exist
so a dialect claim is a test rather than a paragraph.

## Format

    # model:     ams1 | ams2 | ht
    # outcome:   read | notag | foreign
    # source:    where these lines came from
    # verbatim:  yes | abbreviated  (see below)
    #
    # offset | expect | line

`offset` is seconds from the insert edge, or `-` when the log did not record
one. Ordering is significant even without offsets: the lines are in the order
the unit emitted them.

`expect` is the set of bridge patterns that must fire on that line, separated by
spaces, or `.` for none:

| marker     | pattern                 | means                                  |
|------------|-------------------------|----------------------------------------|
| `read`     | `_RFID_READ_OK_RE`      | the unit says a tag read landed        |
| `end`      | `_RFID_CYCLE_END_RE`    | the unit says its scan cycle is over   |
| `foreign`  | `_RFID_FOREIGN_TAG_RE`  | a non-Bambu chip, seen but not opened  |

`verbatim: yes` means the offsets and the text are exactly as the log recorded
them. `verbatim: abbreviated` means the source was a doc that summarised a chain
with arrows and dropped some `[AMS_*]` tags; the wording is the unit's but the
tags and offsets are not all present. Only mark a file `yes` if you pasted from
a log.

## Adding a capture

1. Put the lines in a new file here, in order, one per line.
2. Fill in `expect` with what you believe SHOULD happen -- not what does.
3. Run `pytest tests/test_AFC_bambu_narration_dialects.py`.

A failure means the fixture and the patterns disagree. That is the point: either
the fixture is wrong about the dialect, or the patterns are, and the test says
which line they part company on.

The most useful capture is a **timestamped host log of a single unit alone on
the bus**. Both boxed models answer on `0x0700`, so with an AMS 1 and an AMS 2
on the wire together no narration can be attributed to one of them -- the
address scoping in `_scan_verdict` cannot separate them, and a chain-mate's
sentence will answer for the unit you are watching.
