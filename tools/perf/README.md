# Host planned-time comparison

Run from the repository root. Use this branch's scripts with each firmware tree;
the builder chooses that tree's replay support when present. 449.01 predates
host support, so the fallback adapter stubs I/O/reporting, refuses JSON/settings
input, pads unused command arguments to six slots (the old queue overreads
one-element arrays), and omits unavailable advisor output. Planner, arc, exec,
kinematics and DDA calculations come from the selected tree. Sanitizers stay on.
The machine profile installs identical settings on both sides.

```sh
python3 -I tests/host/planner_timing.py replay tests/hil/fixtures/planner_timing/*.nc --settings tests/hil/fixtures/planner_timing/profiles/bench1.json --source /path/to/A --out /persistent/results/A.json
python3 -I tests/host/planner_timing.py replay tests/hil/fixtures/planner_timing/*.nc --settings tests/hil/fixtures/planner_timing/profiles/bench1.json --source /path/to/B --out /persistent/results/B.json
python3 tools/perf/compare.py /persistent/results/A.json /persistent/results/B.json --expected tools/perf/host-expectations.json
```

Replay records every file, including errors, then exits 1 if any failed. Comparison
requires identical file hashes and profiles; missing, duplicate, failed or invalid
runs cannot become speed wins. Exit 1 means a finding or a planned slowdown over
1%, or a planned speedup over 1% (`FASTER-REVIEW`). This is not the statistical bench gate: no confidence interval, parse speed,
serial round trip, physical pulse width or actual elapsed board time is measured.
HOST-PAUSE and HOLD-RESUME are comments here. The planned duration includes dwell
blocks, not host pauses or operator delays. The expectation manifest marks the
22 fixtures individually; E180 and E017 explain the two observed differences.
Notes never automatically waive a refusal, slowdown or speedup finding. The comparison prints the spec's
E029/N87/N80/N105/N78/N104/E060 review list even when none is exercised.

Observed with bench1 profile on c5024186 vs bc90a6be: 20 matching durations;
05_tiny_collinear 11.976518 vs 3.153175 seconds (approximately 73.7% shorter on B);
09_arcs rejects on A with status 159, succeeds on B. That A refusal is an explicit
finding, not silently labelled as one of the spec's expected error-20 cases.
The dense-CAM fixture takes about 2521 seconds in both host plans, so it is not
a sub-ten-minute bench file under this profile.

Validation uses no board:

```sh
TINYG_REPLAY_BASE=/path/to/A TINYG_REPLAY_CAND=/path/to/B python3 -m unittest discover -s tests/host -p test_replay_compatibility.py
python3 -m unittest discover -s tests/host -p test_perf_compare.py
```

## Why fixtures 05 and 09 differ in the host replay

On c5024186, `plan_line.c:449,468` takes the next block's entry speed from
its previous buffer even after execution cleared that buffer. The trace at the
reset reads previous speed 0, previous buffer EMPTY, and runtime exit speed
589.445 mm/min. `plan_zoid.c:154` also rereads that buffer for a short block.
E180 (7f8639d452bae39e0708d179f3b2ab1f491da584) carries the executed exit speed
when the buffer is empty and removes the second read.

A controlled build of the old generated source, with the adapter unchanged,
reproduces 11.976518 seconds. Changing either read alone still gives 11.976518;
changing both to the E180 behavior gives 3.153175, exactly B's planned time.
The old trace resets every 16 blocks (0.8 mm), not every 80. After the block
exits at 589.445, the next is replanned from zero: its first executed segment is
0.001252 mm/min. This is a discontinuity, not a planned braking ramp. The
short-block threshold is reached after about 16 increments of 36.8403 mm/min;
it is not the planner pool wrapping.

This demonstrates a firmware speed-carrying bug under the host feeding schedule.
It does not establish a 73.7% board speedup. Changing the host queue depth changes
both durations (depth 4: 59.589490 vs 12.634569; depth 16: 14.035057 vs 3.252546).
The adapter's queue progress check and padded command arguments are identical in
all controlled old-source variants. Board streaming and execution timing still
need the A/B measurement.

On c5024186, `plan_arc.c:308-333` computes zero angular travel for coincident
explicit endpoints. Clockwise adds a full turn when travel <= 0; counterclockwise
subtracts a turn only when travel > 0. Thus the counterclockwise circle remains
zero and raises 159. E017 (444e97a3859226c59505ae901cb40cf5a02aa04c) explicitly
recognizes coincident center-format endpoints in either direction. Applying only
that check to the old generated source makes fixture 09 finish in 24.247509
seconds. It does not change fixture 05. This isolates the circle refusal from
the adapter; it is not a full historical-commit bisect.

Controlled-build scripts, generated sources, segment traces and results are in
`/Volumes/External/projects/TinyG-work/perf-host/attribution/` (`check.py`,
`check-entry.py`, `check-trace.py`, and their JSON results). These diagnostic
changes affect generated host copies only; no firmware source was changed.

## Bench collection

Run this on an unloaded bench, alternating A/B with the same SLOT for
one pair and the same settings on one board. No flashing is done by the test.
Install the MSO extra before collecting; retain `--extra mso` on `uv run` so
its environment sync keeps numpy installed. The MSO worker wheel is configured
separately in the bench map.

```sh
uv sync --project tests/hil --extra mso
export PERF_SNAPSHOT=/absolute/path/to/fixed-bench-snapshot.json
FIRMWARE_TAG=A SLOT=1 RT=off uv run --project tests/hil --extra mso pytest tests/hil/tests/diagnostics/test_perf_compare.py --port <port> --allow-motion --diagnostics
FIRMWARE_TAG=B SLOT=1 RT=off uv run --project tests/hil --extra mso pytest tests/hil/tests/diagnostics/test_perf_compare.py --port <port> --allow-motion --diagnostics
```

A sends no `rt` commands. Feedhold affects motion only. `RT=on` enables fi and fg for
the separate B-only cost pass. Each file has its own `perf-*.json` and capture
in the run directory, including failures. Records contain identity, flash
provenance, settings, file hash, acknowledgements, timed stat replies, fixture
pause/hold events, end positions, signed/total motor steps, first-to-last-step
time, minimum high pulse and maximum step rate. Probe replies never count as
G-code acknowledgements. HOST-PAUSE/HOLD-RESUME comments are executed by the
single transport reader. The latter waits for HOLD before timing the resume.

MSO captures all four step/dir pairs. `PERF_SOURCE=saleae` uses the healthy
mapped channels on bench 2, records M4 as missing, and is supplemental evidence.
No source can silently fill a missing motor with zero steps. Digital transitions
are retained for later section/gap analysis; gaps over 5 ms are observations,
not automatic proof that the planner starved. The serial and capture clocks are
not treated as synchronized.

The MSO uses a full, fixed-time capture, not stopped windows that change motion.
`capture-budgets.json` contains twice the larger host A/B duration plus 15 s
under the bench1 profile; perfbench defaults to 600 s. For other settings set
`PERF_CAPTURE_S` and a smaller `PERF_TIMEOUT_S`. Captures without a quiet tail,
failed streams and missing pulses are findings, never speed wins. Fixture 21
is skipped unless `PERF_DENSE=1`; its default capture is over 5000 s. Large
captures need disk/memory on the capture host. These budgets are bounds, not measurements or promises of bench duration.

## Bench comparison

```sh
python3 tools/perf/compare.py --bench /run/A1 /run/B1 /run/A2 /run/B2 --expected tools/perf/host-expectations.json
python3 tools/perf/compare.py --bench /run/B-off /run/B-on --rt-cost
```

Records pair by fixture, bench, capture source and SLOT. Five or more paired
slots are required for a passing statistical review. The deterministic paired
bootstrap reports median B/A (or on/off) and a 95% interval; the entire interval
must fit the allowed slowdown. Limits: motion +1%, lines/s -2%, stat median and
p95 +10%; rt-cost permits motion +2%. Net and total steps must match within one
per motor in every pair. Shorter pulses require review, including sampling
uncertainty; raw maximum rates are printed alongside them. Missing motors,
invalid captures, unmatched or duplicate slots, mismatched files/boards/shared
settings, missing latency data or too few pairs never pass. Only B-only rt/spph/spde/coph controls may lack an A counterpart; missing shared
tokens, differing shared values, and differing status-report field lists fail comparison.
Expected-difference notes are printed but do not waive evidence. A speedup over
1% is FASTER-REVIEW. The command exits 1 for every non-PASS result, including
empty input. Bench 2's three-motor captures remain supplemental findings.

## Motion-only hold timing

`tests/hil/tests/diagnostics/test_e060_timing.py` checks hold/resume and hold/flush
with M3 on. It records command wall times, automatic and polled status reports,
and raw motor/PWM captures. The programmed PWM stays at 80% through both cases;
10% is the off level. Any output drop fails. Resume must produce motor steps;
flush must not. Spindle-transition intervals are null because no transition is
expected. The report includes hold-to-stop and resume-to-motion intervals.
The 20 ms PWM windows and estimated 100 ms capture-start alignment uncertainty
limit command timing precision. Settings are restored afterward. Run with the
same --port/--allow-motion/--diagnostics flags; physical proof requires a bench.

## Advisor trigger checks

`tests/hil/tests/test_feed_limit_causes.py` adds four board cases: fixture13 G61
exact-stop tag and c[2]; an extended fixture05 collinear chain with the single
segment tag alone and c[4]; fixture15 command time c[6] in fs v2; and fixture19
no-input/pending-input gap time plus the disappearance of fl.g on resumed feed.
The pending-input stimulus sends bounded line-mode control/data bursts and drains
all replies before the next burst. It must observe both classes' accounting;
it fails rather than accepting only a no-input gap. Its ability to hold a backlog
long enough on each bench needs qualification. No firmware hook was added.
Three offline negative tests reject conflated causes, absent pending time, stale
gap fields and the wrong command counter. These tests are ready for bench runs,
not evidence that the required causes have already occurred on hardware.

A's settings snapshot excludes post-449 tokens explicitly,
even if installed dialect metadata lists them. Removed controls are not required on B. An offline fake board checks that A never queries these additions. The full
capture holds a host wake assertion, and a detected sleep invalidates the result.

### Firmware identity preflight

Before settings writes, motion or instrument capture, the collector reads `fb`
from the board. Tag A requires 449.01. Tag B requires `PERF_EXPECT_FB` (default
450.04). If `PERF_EXPECT_HEX` is set, it must be a full 64-character SHA256 and
match the port's flash record. A mismatch writes an invalid JSON with the actual
identity and flash record, then stops without resetting or moving that board.
The collector does not build or flash firmware; a failed upstream build cannot
be masked by assigning a different FIRMWARE_TAG.

Each JSON retains the flash record, including commit and hex_sha256, and adds
`known`: a recorded full image hash exists and the record is not legacy/unknown.
A CI artifact can have a known image hash and an empty commit field. This is
flash provenance, not an independent readback of device flash. Expected build
and optional expected hash are recorded alongside it.

Bench comparison requires matching build expectations and known flash hashes,
rejects mixed images/expectations within a tag across slots, requires distinct
images for A/B, and requires the same image for B rt off/on. Older smoke records
without these fields remain exploratory evidence, not automatically passing
pairs under the strengthened checks.


### Legacy 449 reply matching and a common settings snapshot

The driver accepts a nested `r.tid` only after identifying dialect d449 with
449 <= fb < 450. A top-level tid takes precedence; stale nested replies cannot
acknowledge a different request. Firmware 450+ retains strict top-level matching.
The offline session test performs identity bootstrap, original text-mode settings
reads, every real BASELINE write, the persisted settings snapshot, and collector
stat probes using 449.01 reply shapes. Collector probes already accepted nested
tid; the fix is in the session driver used before collection begins.

Flash A and B with one fixed settings snapshot that both builds accept. In
particular, use positive bjd/cjd: restoring A's zero defaults on B can be refused.
The collector records actual readback values, not merely the requested profile.
Comparison rejects changed bjd/cjd, missing shared tokens, unexpected extra
settings and changed report-field lists. It never repairs or normalizes these
values to make a pair pass. Only B-only rt/spph/spde/coph additions are exempt
from requiring a matching A token.


### Required fixed bench settings across every A/B flash

Use ONE saved snapshot of the bench's real wiring and settings, accepted by both
builds, for the entire comparison. Set `PERF_SNAPSHOT` to its absolute path for
both the flash command and collector. Do not replace this file with an automatic
pre-flash backup. The collector requires a full motor map/geometry in this file.

```sh
export PERF_SNAPSHOT=/absolute/path/to/fixed-bench-snapshot.json
uv run --project tests/hil --extra mso python -m tinyg_hil flash --port <port> --hex <image.hex> --perf-snapshot "$PERF_SNAPSHOT"
```

In this mode every successful flash restores the fixed file and reads back its
settings and status-report field list. A restore refusal or readback mismatch
fails the command. `--no-restore` and a conflicting `--restore` file are refused.
If FIRMWARE_TAG=A or B is present, flash requires this mode rather than falling
back to its ordinary just-taken snapshot. Automatic backups are still saved for
recovery, but are never used as the A/B settings source. A snapshot for a different
board is refused before flashing. The flash record alone is not proof that the
settings restore succeeded; the collector checks the settings independently.

Collector preflight checks the actual board against the fixed file before its
settings writes or capture. The pytest session has already applied BASELINE
protocol settings (js=1,jv=4,ee=0,ec=0,tv=1,sv=1,qv=0); these explicit overrides
are recorded. After collector setup a second readback checks its additional
known protocol settings (ej=1,rxm=1,ex=1,qv=1,sv=1,si=250).
Motor map, steps per unit, switches, axis modes/radii, limits, junction values
and every other shared restorable setting must still match the fixed file.
Identity/baud/internal EEPROM-index entries are not restored; known B-only
Runtime controls are omitted on A. Snapshot path/hash and verified
readbacks are recorded. Comparison refuses differing fixed-file hashes or
missing readback verification.

Source finding: c5024186 `firmware/tinyg/config.c:132` compares EEPROM record0
with `cs.fw_build` (449.01). A mismatch calls `_set_defa`, which writes each
initialized default back to EEPROM (`config.c:158-176`). Current450 firmware
uses layout marker10002 (`config.c:242-243`), so downgrading from450 to449 loads
and persists defaults. Upgrading again can migrate those now-default values;
it cannot recover the lost bench configuration. **Every A<->B switch requires a
full restore from the fixed bench snapshot.**

The old flash CLI printed `restore:` only after `snapshot.restore` returned.
An exception during restore (including the now-fixed nested-tid timeout) could
therefore leave no restore line at all. That explains a possible failure path,
not a claim to have verified the exact missing log event from smoke2.

Profile decision: retain the real bench3 motor map and step geometry from the
fixed snapshot for this comparison; do not automatically apply the host bench1
profile, whose axis/motor association differs. At4.444 steps/mm, fixture05 is a
coarse physical check. A later fine-resolution series should use a separately
reviewed fixed snapshot with explicit motor geometry, then rerun BOTH A and B.
Never combine coarse/fine slots. Host planned-time numbers are not predictions
for bench3 unless the host replay uses that same motion profile.

### Opaque user-data words

`uda0..uda3`, `udb0..udb3`, `udc0..udc3`, and `udd0..udd3` are explicitly
not restorable by this harness path. Their original values remain in saved
snapshots and actual collection readbacks. Restore and fixed-snapshot verification
list them under `not_restorable` with their saved values and reason; A/B comparison
prints the excluded token names. This exception does not include motor geometry,
switches, limits, junction settings or arbitrary unknown tokens.

On current firmware, status116 is `STAT_VALUE_TYPE_ERROR`: the JSON value does
not agree with the configured variable type (`error.h:209`). The TYPE_DATA parser
rejects a numeric value before `set_data` runs. Status102 is
`STAT_BAD_NUMBER_FORMAT` (`error.h:194`); the string branch checks a literal `0x`
prefix, unlike the other string paths that unescape the quoted token. Thus blindly
converting the saved hex strings to numbers does not repair the restore. No
firmware/parser change is included here. Opaque user words have no motion effect
and cannot block or invalidate an otherwise matching performance settings set.

### A-only junction-deviation display precision

449.01's cfgArray prints bjd/cjd with precision0, so a stored0.05 reads as0.
Only these two tokens on fb449.01 may be checked against the expected value
rounded to their source precision. B and all other tokens keep strict checks.
`tinyg_hil/data/perf-449-print-precision.json` records c5024186's full commit,
config_app.c SHA256, source rows/line numbers and precision column. Regenerate it
from an unchanged449.01 checkout using:

```sh
python3 tools/perf/extract_legacy_precision.py --source /path/to/edge097 --out tests/hil/tinyg_hil/data/perf-449-print-precision.json
```

Fixed restore always sends the original requested bjd/cjd values on A before
using the reduced-precision post-write check. It never replaces0.05 with0 in
the settings being written. Verification records `reduced_precision` entries
with requested value, actual readback, printed expectation, precision and source
row. Thus the report describes a display-limited check, not proof of the hidden
fractional value from serial readback alone. A mismatch outside the rounded value
still fails. The original snapshot and raw readbacks are retained unchanged.

Comparison accepts the A/B display difference only for the source-listed tokens,
when A's recorded reduced check matches both raw A and the expected B value; it
prints the reduced-precision token names. No unexplained difference, arbitrary
token, different B value, or B-side rounded readback is waived.

### N55: probe-free A/B and separate B round trips

`PERF_PROBES=off` is the default for **every fixture** on A and B. No stat queries
are sent during collection. Automatic status reports (including stat) provide
completion; identical protocol settings are retained on both builds. Compare
steps, motion time, parse-bound lines/s and pulses from these runs. There is no
A streaming RTT baseline: N55 loses priority-line credits while G-code is queued.

Run a separate pass with `PERF_PROBES=on FIRMWARE_TAG=B` for 250 ms stat probes.
A with probes on is rejected before the collector changes settings or starts motion. Probe mode is
recorded, and filenames include it so a separate B pass cannot overwrite a
probe-free record. Report B median/p95 round trips in seconds with:

```sh
python3 tools/perf/compare.py --bench /path/to/run-records --rtt-only
```

This reports absolute per-file/per-slot values, not A/B ratios or a regression
verdict. Invalid captures, missing timing/identity/settings evidence, and receive
credit pressure produce findings. Normal `--bench` comparison excludes the
separate B probe pass, and requires both paired records to have probes off.
`--rt-cost` also uses probe-free B off/on pairs. Older probed motion records are
not interchangeable with the new probe-free conditions.

Every serial record retains rx_credit_initial, rx_credit_min, rx_credit_last and
timestamped rx_credit_rows from every response footer, including priority replies.
Comparison prints credit summaries. Probed records reaching the two-slot reserve
or ending below their initial credit window are findings. This is credit-pressure
evidence, not proof of a leak by itself. N55 (fixed in 572cb835) explains why A
has no baseline; no timeout or failed run is silently waived.

### MSO duration and legacy hold reporting

MSO captures retain 2 MHz (0.5 us sampling) for pulse comparisons. Requested
captures above 500 seconds are capped at the instrument limit; the default stream
timeout is capped capture minus 2 seconds. An explicit timeout beyond that is
rejected. Both requested and actual capture duration and sample rate are recorded.
This fixes perfbench's invalid 600-second capture. It does not make the 42-minute
fixture21 fit: a file exceeding the bounded capture must fail, not pass with a
partial trace. That dense fixture still needs a separate long-capture strategy.
At 1 MHz a 1.4 us pulse has only one or two samples, so lowering the rate would
weaken the pulse-width metric; no sampling-rate change is used here.

On 449.x only, HOLD-RESUME waits for a fresh automatic vel=0 report after !,
then 0.5 seconds settled at zero, before starting the fixture's hold delay.
No query is sent. A velocity rise resets settling. Pre-! zero reports cannot
release the hold. B still requires HOLD stat. The chosen method and confirmation
event are recorded. A completion also requires zero velocity, so stale END status
while moving cannot prematurely finish collection. A needs vel in its existing
status-report fields; the collector refuses if absent instead of changing them.

449.01 canonical_machine.c:177 returns stored READY/STOP/END before consulting
motion state. The supplied fixture20 trace reports stat4 even at vel800 and then
at vel0 in hold. N99 commit046666db changes this reporting path to consult runtime,
runnable blocks and loader ownership before returning stored idle states (merged
81969e89). This identifies the stale-reporting fix, not the exact earlier event
that left the stored END state behind in that bench run. No claim of a board bisect
is made. Compare prints this explanation for fixture20, including A's additional
settle time; failures or elapsed-time differences are not waived.

### Revised pulse and step rules (maintainer, 2026-11-11)

Net steps must match within one step per motor. Total pulses are retained and
printed, but equal net and equal reversal-leg counts allow different totals:
fractional-step phase can contribute one pulse on each repeated reversal leg.
See `TinyG-work/perf-host/bench3-attribution/report.txt`. A leg is a consecutive
run of pulses in one direction; direction pin changes with no pulse do not create
legs. Different leg counts fail; missing leg evidence remains a finding.

Pulse shortening fails only at a difference of at least one recorded sample
period. Both rates, raw widths and the threshold are printed. If rates differ,
the coarser sample period is used. Missing sampling rate remains a finding.
New collectors record the rate, channel mapping and leg counts. For old MSO
records, the CLI reads the saved capture and reconstructs legs in memory. If the
mapping is absent it uses the matching bench in
`~/.config/tinyg-hil/benches.json`; every reconstructed motor total and signed net
must match the record exactly before accepting that mapping. No old record or
capture is rewritten. Missing captures or mismatched maps cannot silently pass.

Speedups over1% remain FASTER-REVIEW. Attribution is E180 for05, E089 startup
lookahead for14/16/18/19, and explicitly unattributed for other fixtures. These
labels do not waive failed captures, timing limits or confidence-interval findings.

### Failed capture cleanup

A failed stream exits the MSO context through `Capture.abort()`. Workers run in
separate process sessions, so cleanup terminates only the launched worker and its
recorder descendants. It waits for the worker, kills remaining members if needed,
and waits for the recorder process group to disappear before propagating the
error. The cached persistent worker is discarded, so the next capture starts a
fresh session. The same cleanup covers capture-start and finish failures. It does
not search for or kill unrelated MSO processes.

### Credit recovery in a priority reply

A stat reply refreshes the sender's receive-credit estimate without acknowledging
any G-code. When older data holds priority fragments in RX, the final data reply
can report zero credits; a later probe can be the only reply announcing that
those fragments are now free. Ignoring that recovery stalls the host with unsent
lines even though the board has stopped with an empty RX queue.

Serial records include `sent`, `sent_rows` (index, time, line), `acked`, `pending`
and `unsent`. These distinguish data never written from data awaiting a reply.
The 2026-11-12 short-arcs failures sent13/22 lines and received13 data replies;
lines13..21 were never sent. The last executed endpoint was X10/Y0, not X24/Y0.
This was a collector credit-refresh bug, not evidence of firmware dropping lines.
Minimum credit remains reported; temporary credit pressure is not itself proof
of an N55 leak when credit later recovers.

### Configurable B probe passes

`PERF_PROBES=on` defaults to `PERF_PROBE_KIND=stat PERF_PROBE_MS=250`.
Use `PERF_PROBE_KIND=qr PERF_PROBE_MS=500` for the job-stream polling interval,
or `PERF_PROBE_KIND=qr PERF_PROBE_MS=100` for the faster drain/pause interval.
The interval must be finite and positive. Both kinds remain B-only. These passes
send tid-tagged queries at a fixed interval with at most one unanswered query;
they do not reproduce every adaptive polling rule in run.

Queue replies return queue space, not machine state. The collector uses automatic
status reports for completion and hold detection during qr passes, and still
refreshes RX credit from each validated reply's footer. Neither kind acknowledges
a pending G-code line. Every record carries `perf_probe_kind` and `perf_probe_ms`;
filenames include e.g. `probes-on-qr-500ms` (also included for probes-off records).
Serial results contain `probe_kind`, `probe_ms`, `probe_median_s`, `probe_p95_s`,
and each query's value. The legacy `stat_*` timing fields are null for qr passes.
`compare.py --bench --rtt-only` reports kind and interval on each absolute result;
no A baseline or mixing of stat and qr latency is implied.

For motion-only firmware, fixed-snapshot restoration discards the three retired
output-pause controls and records their names in `removed_controls`. Recreate a
fixed snapshot after upgrading; historical 450.04 records remain readable.
