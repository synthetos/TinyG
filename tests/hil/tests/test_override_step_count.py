"""Live feed override never loses or adds travel: physical step counts, read through the Atmel-ICE.

test_hook.steps[m] (test-hooks build) is the signed count of step pulses the DDA issued, never rewritten by G28.3,
G92 or a flush. For each case: on the finest X/Y/Z axis, a relative move of about 80 mm (a whole number of steps) at F600 with the
override changed while it runs, then the same move back, three times. Every move must land within 1 step of its target and the
position must stay within 1 step of the start after every round trip (exact counts are recorded).

Feed-Override wiki page: "The factor scales the programmed feed rate" (speed only; the path is unchanged). The ICE
is read only at rest, before and after each move, so it never pauses the motion being measured. The scope test
tests/diagnostics/test_override_step_rate_mso.py checks the same counts on the pulses plus the speeds (bench 3).
"""
import time

import pytest

pytestmark = [pytest.mark.ice, pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"),
              pytest.mark.timeout(420)]

DIST, FEED = 80.0, 600.0
ROUND_TRIPS = 3

# (requests during the move as (seconds after the line, factor or "hold"/"resume"), x speed limit mm/min)
CASES = {
    "none": ([], 1200),
    "unity": ([(2.0, 1.0)], 1200),
    "half": ([(2.0, 0.5)], 1200),
    "one_and_half": ([(2.0, 1.5)], 1200),
    "half_then_double": ([(2.0, 0.5), (4.0, 2.0)], 1200),
    "capped": ([(2.0, 2.0)], 800),
    "change_during_hold": ([(2.0, "hold"), (3.0, 1.5), (3.5, "resume")], 1200),
}


@pytest.mark.parametrize("case", list(CASES))
def test_override_step_count(tg, machine, ice, keep, record, case):
    if "test_hook" not in ice.symbols.sym:
        pytest.skip("needs the test-hooks build (physical step count)")
    # The finest axis gives the most sensitive count; the distance is a whole number of steps so that the
    # outbound count is exact and every round trip must land back on the starting step.
    axes = [(machine.steps_per_unit(a), -i, a) for i, a in enumerate(("x", "y", "z")) if machine.motors_for(a)]
    if not axes:
        pytest.skip("no motor mapped to X, Y or Z")
    spu, _, axis = max(axes)                      # ties go to X, then Y, then Z
    motors = machine.motors_for(axis)
    expected = round(DIST * spu)
    dist = round(expected / spu, 4)
    if abs(dist * spu - expected) > 0.02:
        pytest.skip(f"no 4-decimal distance near {DIST} mm is a whole number of steps on {axis} ({spu} steps/mm)")
    syms = [f"test_hook.steps[{m - 1}]" for m in motors]
    requests, limit = CASES[case]

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

    def run(line, timed):
        start = counts()
        tg.guard.check_gcode(line, allow="override step-count move")
        tg.send_line(line)
        t0 = time.monotonic()
        for at, what in timed:
            time.sleep(max(0.0, at - (time.monotonic() - t0)))
            if what == "hold":
                tg.send_raw(b"!")
            elif what == "resume":
                tg.send_raw(b"~")
            else:
                assert tg.send_json({"mfo": what}, check=True).status == 0
        tg.wait_idle(timeout=60)
        end = counts()
        return [e - s for s, e in zip(start, end)]

    A = axis.upper()
    trips = []
    try:
        for _ in range(ROUND_TRIPS):
            out = run(f"G1 {A}{dist:g} F{FEED:g}", requests)
            mfc = float(tg.get("mfc"))
            tg.set("mfo", 1)
            back = run(f"G1 {A}-{dist:g} F{FEED:g}", [])
            trips.append({"outbound": out, "return": back, "mfc": mfc})
    finally:
        tg.set("mfo", 1)
        tg.send_gcode("G90", check=True)
    running, total = [], [0] * len(motors)
    for t in trips:
        total = [total[i] + t["outbound"][i] + t["return"][i] for i in range(len(motors))]
        running.append(list(total))
    summary = {"case": case, "axis": axis, "motors": motors, "steps_per_mm": spu, "distance_mm": dist,
               "expected": expected, "trips": trips, "net_after_each_trip": running,
               "exact": all(abs(s) == expected for t in trips for s in t["outbound"] + t["return"])}
    record("override_step_count", summary)
    # Agreed bound: each move within 1 step of its target per motor (the starting substep phase can move a count
    # by one), and the signed position back within 1 step of the start after every round trip, so a residual
    # that grows from trip to trip fails. Exact equality is recorded, not required.
    for t in trips:
        for steps in t["outbound"] + t["return"]:          # sign follows motor polarity; the net catches direction
            assert abs(abs(steps) - expected) <= 1, f"a move missed its target by more than 1 step: {summary}"
    for net in running:
        assert all(abs(n) <= 1 for n in net), f"the position drifted more than 1 step from the start: {summary}"
