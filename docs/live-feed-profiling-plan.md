# Override interrupt qualification

The frozen BODY and publication images are not interrupt-timing qualification.
A separate profiling build is required before the final gate. No instrument or
board is operated by the development tests.

Use the unallocated TCC1 (TIMER_5), with its output channels and interrupts off,
as a 16-bit elapsed-time counter at F_CPU/8: 0.25 us per tick, 16.384 ms full
range. Select exactly one measurement point per build, so a nested interrupt
cannot restart another point's counter. Clear the counter and overflow flag at
entry; record elapsed ticks and the overflow flag at exit. Any overflow is a
saturated/invalid sample, never a small modulo duration. Reject a run with one.
Keep sample count, maximum, overflow count and the maximum's motion section in
RAM. Read the results only after the move through working ICE on bench 1.

Separate points:

- Whole exec interrupt, including private-plan acceptance and the first segment
  of the transition. Include high-priority preemption in the reported duration.
- DDA interrupt, especially the terminal segment and load handoff.
- Each interrupt-masked override window: producer snapshot, producer publication,
  and consumer validation/copy. Measure the two producer windows separately.

Build the baseline and candidate with the same compiler, linker relaxation and
selected instrumentation. Also measure an empty bracket to report instrumentation
cost. Inspect the disassembly to identify code before/after the C bracket; report
that prologue/epilogue cost explicitly rather than calling the C bracket the
complete interrupt. Check the maximum masked interval against the active step
period. Do not subtract an average interrupt cost from a worst-case sample.

Exercise: both speed directions, minimum/maximum factors, capped feed, repeated
requests, requests while the transition is being prepared, hold/resume, flush,
kill and alarm, and high step rate on all active motors. Include the unchanged
100% path. The bench 3 MSO checks real pulse widths, timing and step totals in
parallel with the counter qualification; it is needed to catch a late or missed
step deadline that an interrupt-body duration alone would not establish.

This profiling mode must be separate from the full hooks image: the normal and
full hooks release candidates must still fit application flash independently.
Do not weaken the 196608-byte link limit. Stack high-water belongs on bench 1
with working OCD, and must include the deepest producer/report path plus nested
interrupts. The normal/hooks floors remain 1536/1024 bytes.

Size plan for lifecycle: remove the unshipped record-only spindle override path
(the feed feature must leave spindle unity), share the override NV conversions,
and retain size optimization only for configuration and new reporting wrappers.
Keep motion/exec/interrupt optimization unchanged. Measure again after the
motion-only hold branch is integrated; do not assume the removal yields savings,
since it also adds migration code. If either final image does not fit, it is not
READY. Re-run the 100% performance comparison on the final integrated image.

## Building and reading the counters

Build one site at a time with `make -C firmware/tinyg MOTION_PROFILE=N`:

| N | Timed body |
|---|---|
| 1 | AVR exec ISR, including higher-priority preemption |
| 2 | AVR DDA ISR, ordinary and terminal/load paths |
| 3 | Main-loop future-boundary snapshot under the interrupt guard |
| 4 | Main-loop ready-byte validation/publication under the interrupt guard |
| 5 | Exec's boundary validation and plan acceptance under the interrupt guard |

The images are in `build-profileN`. They cannot be combined with `TEST_HOOKS`.
The ELF symbol `motion_profile` contains `calls`, `overflows`, `maximum_ticks`,
`site`, and `section_at_maximum` (the motion section at the end of that sample). Read it after stopping through an ICE with working OCD access.
At 32 MHz each tick is 0.25 us. Any nonzero overflow count invalidates the bound;
the maximum saturates at 65535 rather than showing a wrapped short measurement.
Board reset clears the record. Each image instruments only one site, so a
preempting interrupt cannot restart another site's counter.

The count excludes ISR prologue/epilogue and the timer setup / record accounting
instructions. In particular, sites 3–5 keep interrupts masked while recording:
add the worst-case cycles of those excluded instructions from that ELF's
disassembly when assessing the actual profiling image's masked interval. Do not
report the timer count alone as the full interrupt latency or masked interval.
The record itself adds 12 bytes of RAM only in the profiling image. The normal
and hooks images compile out all instrumentation. Match the baseline's compiler,
link relaxation and site bracket before comparing it with these measurements.
