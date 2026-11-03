# Host corpus replay

Run files through the real G-code parser, canonical machine, arc generator and
line/trapezoid planner. A logical scheduler drains planner buffers; this does not
run a board, the segment executor, the DDA, or interrupt timing.

From the repository root, using Python 3 and a host C compiler with ASan/UBSan:

```sh
python3 tests/host/corpus_replay.py --self-test \
  --output /tmp/replay --source /path/to/candidate/worktree \
  /path/to/generated /path/to/corpus
```

`--source` defaults to this checkout. It reads that source tree without editing
it. It must contain the runtime/feed-limit interfaces used by this fixture; an
incompatible revision fails the build instead of falling back to different code.
Directories are searched recursively for `.nc`, `.ngc`, `.gcode` and `.tap`;
explicit filenames can have any suffix. Each file runs in a fresh process.
Use a dedicated output directory: generated build files and reports are replaced.

`--baseline /tmp/previous/per-file.json` compares refusal statuses for identical
input content. This detects changes in acceptance; it does not decide whether a
foreign dialect is valid TinyG input. `--self-test` checks accepted motion, arcs,
queued commands, deliberate syntax errors, and synthetic numeric/duration faults.
`python3 -m unittest discover -s tests/host -p test_corpus_replay.py` also checks
fixture coverage and report bookkeeping.

## Results

- `summary.json`: source commit and dirty flag, counts, host and timing.
- `per-file.json`: SHA-256, every refusal's physical line/status, alarms, numeric
  findings, coverage notices, and time per file.
- `diagnostics.jsonl`: exact file, physical line, original input and diagnostic.
- `logs/`: full stdout/stderr per file. `build/` contains generated C, the compiler
  command, build log and executable; it can be inspected or run directly.

Exit 1 means a crash, timeout, incomplete file, alarm, numeric finding, uncovered
fixture path, or baseline status change. Ordinary refusals alone do not fail the
run: corpora commonly contain unsupported tool tables, macros and other dialects.
An expected invalid arc can alarm and cause later lines to be refused. Interpret
the first alarm before treating its subsequent refusals as independent defects.
No general independent G-code validity oracle is claimed.

The five-second watchdog covers each line and the final drain; `--timeout` bounds
the entire file. Sanitizer errors terminate the file. A drain that cannot release
a buffer is a fixture failure rather than a successful replay.

## Configuration and execution

Every file starts with `G21 G90 G17 G94 G92.1 G54` and `F600`. The declared machine
has XYZ feed/velocity caps of 16000 mm/min, A caps of 3600 deg/min, XYZ/A standard,
B/C disabled, jerk settings 500, junction deviation 0.05, junction acceleration
100000, and chordal tolerance 0.01 mm. Motors map to XYZA at 1280 steps/unit.
Offsets are zero and soft limits are off. This is a fixed test configuration,
not a user's EEPROM image or the firmware's selected default profile.

Whole buffers drain when free buffers fall below planner headroom, retaining
lookahead. Arc callbacks run to completion. Queued command callbacks run in
order, including spindle/program-end commands; M400 drains preceding work.
Program pauses receive an eventual simulated operator continuation. Dwell time,
feedhold timing, physical steps, wall-clock starvation and serial flow are not
simulated. Homing/probing requests are reported as uncovered fixture paths.

The complete G-code/canonical/arc/planner source files are compiled, along with
utilities and spindle/runtime/feed-limit code. JSON/text parsing and generic
get/set functions are extracted from the selected source. The configuration
table is deliberately limited to the corpus's xjm/ja/xvm/si/runtime tokens and
report queries. A real firmware token absent from this table generates
`UNMODELED_CONFIG` and fails coverage, rather than being called a firmware
refusal. Reporting-only getters generate `REPORT_STUB`; their payloads, report
selection and side effects are outside this tool's coverage. EEPROM, pins,
clocks and transport output are stubs. Extend the adapter before using new
settings-heavy corpora to draw conclusions about those settings.

Firmware floating literals and math calls use float forms to approximate AVR
binary32. Host libm, host integer promotions and the logical drain are not
bit-identical AVR execution. Sanitizers help find bad accesses/conversions but
do not make the fixture a cycle-accurate simulator.

## Numeric screens

Queued plans are checked after input and before consumption for finite,
nonnegative lengths, speeds, limits, jerk terms and move times, plus finite
direction and target vectors. Signed vectors and coordinates are legal.
`naiive_move_time = +inf` with both endpoint velocities zero is logged separately:
the planner intentionally permits that diagnostic division by zero. It is not
an infinite execution duration. Other nonfinite values remain findings.

An independent double-precision estimate reconstructs steady motion time from
displacement, G94/G93 feed, axis caps and a conservative 45 kHz motor capacity,
with a minimum-segment floor. A move-time excess of 10x is flagged. Profile time
is reconstructed from head/body/tail lengths and endpoint velocities and compared
with 10x `[steady time + 4*cbrt(length/(2*jerk))]`. This includes a conservative
rest-to-rest allowance; it is an outlier screen, not a second exact planner or a
proof that every acceleration profile is correct.

Timing excludes compilation. Wall timing includes subprocesses, sanitizers, log
capture and report parsing; C CPU timing includes parsing/planning/draining and
stdout generation. These are host fixture throughput figures, not board speed.
