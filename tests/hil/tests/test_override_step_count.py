"""Live feed override never loses or adds travel: physical step counts, read through the Atmel-ICE.

test_hook.steps[m] (test-hooks build) is the signed count of step pulses the DDA issued (en.steps_run with the
step sign, so it follows signed travel independent of output-pin polarity), never rewritten by G28.3, G92 or a
flush. For each case: on the finest X/Y/Z axis that has one, a positive relative move of about 80 mm (a whole number of
steps on every mapped motor; bench 1's X pairs 1280 and 4.44 steps/mm, so it moves 80.1 mm) at F600 with the override changed while it runs, then the same move back, three times.

Agreed bound (the starting substep phase can move a count by one): each outbound move within 1 step of +target and
each return within 1 step of -target, per motor; the signed position within 1 step of the start after every round
trip. Three trips show the residual stays bounded over those trips; exact counts are recorded.

Feed-Override wiki page: "The factor scales the programmed feed rate" (speed only; the path is unchanged); "During
a feedhold a new factor is accepted and used after resume. Changing the factor never resumes motion by itself."
The ICE is read only at rest, before and after each move, so it never pauses the motion being measured. The scope
test tests/diagnostics/test_override_step_rate_mso.py checks pulses and speeds on bench 3.
"""
import time

import pytest

from tinyg_hil.override_steps import HOLD_COMPLETE, pick_axis, wait_for_hold, within

pytestmark = [pytest.mark.ice, pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"),
              pytest.mark.timeout(420)]

DIST, FEED = 80.0, 600.0
ROUND_TRIPS = 3

# Events during the outbound move, in order: (earliest seconds after the line, action). An action is a factor, or
# "hold" (send ! and wait for a completed hold), "held" (confirm still holding with the new factor requested),
# or "resume" (send ~).
CASES = {
    "none": ([], 1200),
    "unity": ([(2.0, 1.0)], 1200),
    "half": ([(2.0, 0.5)], 1200),
    "one_and_half": ([(2.0, 1.5)], 1200),
    "half_then_double": ([(2.0, 0.5), (4.0, 2.0)], 1200),
    "capped": ([(2.0, 2.0)], 800),
    "change_during_hold": ([(2.0, "hold"), (0, 1.5), (0, "held"), (0, "resume")], 1200),
}


def _spm(tg, m):
    return 360.0 * tg.get(f"{m}mi") / (tg.get(f"{m}sa") * tg.get(f"{m}tr"))


@pytest.mark.parametrize("case", list(CASES))
def test_override_step_count(tg, machine, ice, keep, record, case):
    if "test_hook" not in ice.symbols.sym:
        pytest.skip("needs the test-hooks build (physical step count)")
    mapped = [a for a in ("x", "y", "z") if machine.motors_for(a)]
    if not mapped:
        pytest.skip("no motor mapped to X, Y or Z")
    pick = pick_axis([(a, [_spm(tg, m) for m in machine.motors_for(a)]) for a in mapped], DIST)
    if pick is None:     # a gate bench must measure something: no silent skip (bench 1 skipped this until 2026-11-15)
        pytest.fail(f"no X/Y/Z axis has a whole-step distance near {DIST} mm on every mapped motor")
    axis, dist, targets, spms = pick
    motors = machine.motors_for(axis)
    syms = [f"test_hook.steps[{m - 1}]" for m in motors]
    events, limit = CASES[case]
    factors = [a for _, a in events if not isinstance(a, str)]
    final_factor = factors[-1] if factors else 1.0

    keep(f"{axis}vm", f"{axis}fr")
    tg.set(f"{axis}vm", limit, allow="speed limit for the override step count")
    tg.set(f"{axis}fr", limit, allow="feed limit for the override step count")
    for tok in ("mfoe", "m48e"):
        tg.set(tok, 1)
    tg.set("mfo", 1)
    tg.send_gcode("G21 G91 G94 G64 M48", check=True, allow="relative mode, program override permission")

    def counts():
        tg.wait_idle()
        return [int(v) for v in ice.peek_many(syms).values()]

    hold_log = {}

    def run(line, timed):
        start = counts()
        tg.guard.check_gcode(line, allow="override step-count move")
        tg.send_line(line)
        t0 = time.monotonic()
        for at, what in timed:
            time.sleep(max(0.0, at - (time.monotonic() - t0)))
            if what == "hold":
                tg.send_raw(b"!")
                hold_log["reached_s"] = round(wait_for_hold(lambda: tg.get("hold"), timeout=5.0), 3)
            elif what == "held":
                time.sleep(0.5)                   # changing the factor must not resume motion by itself
                hold_log["held_after_change"] = tg.get("hold")
                hold_log["mfo_while_held"] = float(tg.get("mfo"))
                assert hold_log["held_after_change"] == HOLD_COMPLETE, f"motion resumed without ~: {hold_log}"
                assert hold_log["mfo_while_held"] == pytest.approx(factors[-1]), hold_log
            elif what == "resume":
                tg.send_raw(b"~")
            else:
                assert tg.send_json({"mfo": what}, check=True).status == 0
        tg.wait_idle(timeout=60)
        end = counts()
        return [e - s for s, e in zip(start, end)]

    trips = []
    try:
        for _ in range(ROUND_TRIPS):
            out = run(f"G1 {axis.upper()}{dist:g} F{FEED:g}", events)
            mfc = float(tg.get("mfc"))
            tg.set("mfo", 1)
            back = run(f"G1 {axis.upper()}-{dist:g} F{FEED:g}", [])
            trips.append({"outbound": out, "return": back, "mfc": mfc})
    finally:
        tg.set("mfo", 1)
        tg.send_gcode("G90", check=True)
    running, total = [], [0] * len(motors)
    for t in trips:
        total = [total[i] + t["outbound"][i] + t["return"][i] for i in range(len(motors))]
        running.append(list(total))
    summary = {"case": case, "axis": axis, "motors": motors, "steps_per_mm": spms, "distance_mm": dist,
               "targets": targets, "trips": trips, "net_after_each_trip": running, "hold": hold_log,
               "final_factor": final_factor,
               "exact": all(t["outbound"][i] == targets[i] and t["return"][i] == -targets[i]
                            for t in trips for i in range(len(motors)))}
    record("override_step_count", summary)
    for t in trips:
        assert t["mfc"] == pytest.approx(final_factor), f"factor in effect is not the one requested: {summary}"
        for i in range(len(motors)):
            assert within(t["outbound"][i], targets[i]), f"outbound move missed +target by more than 1: {summary}"
            assert within(t["return"][i], -targets[i]), f"return move missed -target by more than 1: {summary}"
    for net in running:
        assert all(within(n, 0) for n in net), f"the position drifted more than 1 step from the start: {summary}"
