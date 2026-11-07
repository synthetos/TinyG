"""N87: a move shorter than the minimum block time is dropped when nothing follows it.

plan_line.c mp_aline() refuses a move it estimates under MIN_BLOCK_TIME (2.5 ms) with STAT_MINIMUM_TIME_MOVE;
canonical_machine.c _queue_line() turns that into STAT_OK, and cm_finalize_move() still advances the model
position. The planner position (mm.position) is not advanced, so the next move absorbs the missing distance;
when no move follows (end of a job, before a dwell, M5, G92 or a probe) the tool stays short. At rest the board
reports the model position, so the position report claims the endpoint.

Found on bench 3 (edge-next 01fc17ee): gen2-rotary-005 ended 20 Z steps (16 um) and 4 Y steps short; the last
line moved A 0.05 deg at F2400 (about 1.4 ms). The firmware's own step count agreed with mr.position, which
stopped at the second-to-last line, while mpo reported the last.

mp_aline() estimates a short move from the entry speed of the block that is running, not the last one queued.
Here one axis (jerk 5000, so the moves reach their feed) gets two 1 mm moves and then a 1.25 ms move (at F2400 or the axis feed limit), queued while the
first move still runs at speed, with nothing after it. (After a single move the running block exits at zero and
the same short move is estimated over the minimum and runs.) The runtime position and the firmware step count (over the ICE, read only
at rest) must reach the commanded 2.05 mm.

A G28.3 queued right behind the short move rebases the model and the planner at once: the short move must still
run before the origin is set (test-hooks build: test_hook.steps counts pulses and G28.3 does not rewrite it).
"""
import pytest

pytestmark = [pytest.mark.motion, pytest.mark.ice, pytest.mark.dialect("d449"),
              pytest.mark.bug("N87", on=[]), pytest.mark.timeout(6000)]   # includes waiting for the board lock

WHY = "N87 short last move"
MOVE_MS = 1.25                                     # the short move at the feed actually used


def _setup(tg, keep):
    motors = {m: tg.get(f"{m + 1}ma") for m in range(4)}
    spu = {m: 360 / tg.get(f"{m + 1}sa") * tg.get(f"{m + 1}mi") / tg.get(f"{m + 1}tr")
           for m, ax in motors.items() if ax in (0, 1, 2)}
    m = max(spu, key=spu.get)                      # the finest linear motor: the short move is several steps
    axis = "xyz"[motors[m]]
    keep(f"{axis}jm")
    tg.set(f"{axis}jm", 5000, allow=WHY)             # the bench 3 soak's jerk: the moves reach their feed
    feed = min(2400.0, tg.get(f"{axis}fr"))          # the axis feed limit caps F
    short = round(feed / 60000 * MOVE_MS, 4)
    if spu[m] * short < 4:
        pytest.skip(f"the short move is {spu[m] * short:.1f} steps")
    return m, motors[m], axis, spu[m], feed, short


def _queue_moves(tg, axis, feed, short):
    for _ in range(2):
        tg.send_gcode(f"G1 {axis}1 F{feed:g}", check=True, allow=WHY)
    tg.send_gcode(f"G1 {axis}{short}", check=True, allow=WHY)   # planned while the first move still runs


def test_short_last_move_reaches_its_endpoint(tg, ice, keep, record):
    m, ax, axis, spu, feed, short = _setup(tg, keep)
    names = [f"mr.position[{ax}]", f"en.en[{m}].encoder_steps", f"en.en[{m}].steps_run"]

    def snapshot():
        tg.wait_idle()
        v = ice.peek_many(names)
        return {"runtime": v[names[0]], "steps": v[names[1]] + v[names[2]], "mpo": tg.get(f"mpo{axis}")}

    tg.send_gcode("G21 G91", check=True)
    try:
        before = snapshot()
        _queue_moves(tg, axis, feed, short)
        after = snapshot()
    finally:
        tg.send_gcode(f"G1 {axis}{-2 - short:.4f} F{feed:g}", allow="N87 return to the start")
        tg.send_gcode("G90", allow="N87 restore absolute mode")
        tg.wait_idle()
    want = 2 + short
    got = {"runtime_mm": after["runtime"] - before["runtime"], "steps": after["steps"] - before["steps"],
           "mpo_mm": after["mpo"] - before["mpo"]}
    record("n87", {"motor": m + 1, "axis": axis, "steps_per_mm": spu, "feed": feed, "short": short,
                   "before": before, "after": after, "got": got})
    print("N87", axis, got)
    assert abs(got["mpo_mm"] - want) < 0.002, f"reported travel {got['mpo_mm']}"
    assert abs(got["runtime_mm"] - want) < 0.5 / spu, f"runtime travel {got['runtime_mm']:.4f} mm, want {want}"
    assert abs(got["steps"] - want * spu) <= 1.1, f"{got['steps']} steps, want {want * spu:.1f}"


def test_short_move_runs_before_g28_3(tg, ice, keep, record):
    if "test_hook" not in ice.symbols.sym:
        pytest.skip("needs the test-hooks build (physical step count)")
    m, ax, axis, spu, feed, short = _setup(tg, keep)
    names = [f"test_hook.steps[{m}]", f"en.en[{m}].steps_run"]

    def snapshot():
        tg.wait_idle()
        v = ice.peek_many(names)
        return {"steps": v[names[0]] + v[names[1]], "mpo": tg.get(f"mpo{axis}")}

    tg.send_gcode("G21 G91", check=True)
    before = snapshot()
    try:
        _queue_moves(tg, axis, feed, short)
        tg.send_gcode(f"G28.3 {axis}0", check=True, allow=WHY)   # queued behind the short move
        after = snapshot()
    finally:
        tg.send_gcode(f"G1 {axis}{-2 - short:.4f} F{feed:g}", allow="N87 return to the start")
        tg.wait_idle()
        tg.send_gcode(f"G28.3 {axis}{before['mpo']:.4f}", allow="N87 restore the machine position")
        tg.send_gcode("G90", allow="N87 restore absolute mode")
        tg.wait_idle()
    want = (2 + short) * spu
    got = {"steps": after["steps"] - before["steps"], "mpo": after["mpo"]}
    record("n87_g28_3", {"motor": m + 1, "axis": axis, "steps_per_mm": spu, "feed": feed, "short": short,
                         "before": before, "after": after, "got": got})
    print("N87 G28.3", axis, got)
    assert abs(got["mpo"]) < 0.002, f"mpo {got['mpo']} after G28.3 {axis}0"
    assert abs(got["steps"] - want) <= 1.1, f"{got['steps']} steps before the origin, want {want:.1f}"
