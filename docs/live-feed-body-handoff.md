# Live feed override: long straight move handoff

The main loop prepares a private speed change for a future cruise segment. The
current increment handles a straight G1 in units per minute, with no rotary
travel. It leaves the programmed endpoint and the next block's entry speed
unchanged. Rapids, arcs, probing, homing, inverse time and dwells are excluded.

The snapshot selects a boundary 32 segments ahead. It records the block's
incarnation, slot, segment number, position, speed, remaining segment count,
segment duration, request generation and cancellation generation. Floating-point
planning runs with interrupts enabled. Publication rechecks the live block and
request, then sets one ready byte. At the selected boundary, exec validates the
snapshot again and copies the private head/body/tail lengths and waypoints. It
does not search or replan the queue. The normal segment preparation code then
runs the transition.

A newer request, a completed block, an expired boundary, a feedhold, a queue
flush, a job kill or an alarm rejects the pending plan. Feedhold sequencing runs
before the producer in the main loop. A request that cannot fit a complete
transition in the remaining move is deferred; it does not alter a junction to
make room. The transition keeps axis velocity, motor pulse-rate and jerk caps.

Host replay checks cover 600 to 300 and 600 to 900 mm/min, measured transition
jerk, unchanged endpoint coordinates, cancellation during planning, block
completion during planning, expiration, and hold/resume after acceptance.
Each motor must stay within one step of its commanded endpoint, with exact
agreement between emitted pulses and the firmware encoder count. The replay
checks plain motion, constant 50%/150% requests, mid-move changes, and a 50-move
out-and-back sequence with boundary requests and several changes within each
move. It checks every endpoint and every move's pulse increment, not only the
final return to zero. The same consistency checks run after hold/resume with an
accepted override. The allowance is the established one-step DDA residual; it
never grows with the number of moves or requests. No stepper changes were made.

The HIL test checks sustained speed and the reported endpoint. It does not prove
pulse counts or interrupt latency. Those require an instrumented bench run.
Committed-factor publication, the versioned override frame, selectable status
fields and ordered M48/M49/M50 permissions belong to the following increment;
the current committed readout still reports unity.

The portable build now relaxes linker branches/calls and enforces the 192 KiB
application limit, excluding the bootloader. Both normal and hooks images must
fit that limit before any bench image is offered. The arithmetic compiler
settings are unchanged. Bench qualification must include interrupt timing and
the 100% performance comparison with this link setting.
