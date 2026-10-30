"""E030: resuming from a feedhold re-runs the held block's old trapezoid from the hold point, overshooting.

edge plan_exec.c:236-238 only patches `bf->block_state`, `bf->length` and `bf->entry_vmax` at the hold point;
mp_calculate_trapezoid never runs again (bf->replannable=false at :183, _plan_block_list stops at :439 of
plan_line.c, mp_plan_hold_callback is declared in planner.h:298 but not defined). On resume :189-205 reload the
stale head/body/tail lengths and build waypoints from the hold point, so the move travels its full original
length again. Master replans in mp_plan_hold_callback (controller.c:165, plan_line.c:641-747).

At rest the board reports the model position (canonical_machine.c:140-147 switch to MODEL on MOTION_STOP),
which hides where the runtime really ended. While moving it reports the runtime position, so:
- the overshoot is caught by sampling posx while the resumed move runs, and
- the runtime end point is measured with a slow probe move afterwards: the lowest posx sampled while the
  probe moves is where the previous motion really stopped (within about F30 x one sample period).
"""
import time

import pytest

MOVE = "up to 6 mm +X at F120-F900 with feedholds and resumes, a 0.5 mm probe move, then back"
TOL = 0.05          # mm: probe sampling resolution plus margin; the bug's error is millimetres


def _wait_hold(tg, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if tg.get("hold") == tg.dialect.hold("HOLD") and tg.get("vel") == 0:
            return
    raise TimeoutError("feedhold did not reach HOLD")


def _run_until_stopped(tg, timeout=10.0):
    """Sample runtime posx until the queue is empty and posx stops changing; return the peak.

    vel is not used: without the E033 fix the cycle never ends after G1, so vel keeps the last
    segment's value at rest.
    """
    peak, last, end = None, None, time.monotonic() + timeout
    while True:
        body = tg.send_json({"posx": None, "qr": None}, check=True).body
        peak = body["posx"] if peak is None else max(peak, body["posx"])
        if body["qr"] == tg.planner_buffers and body["posx"] == last:
            return peak
        last = body["posx"]
        assert time.monotonic() < end, "motion did not finish"
        time.sleep(0.02)


def _runtime_end(tg):
    """Where the last motion really stopped: lowest runtime posx while a slow +0.5 mm probe move runs."""
    tg.send_gcode("G91 G1 X0.5 F30", check=True, allow=MOVE)
    low, end = None, time.monotonic() + 5
    while time.monotonic() < end:
        body = tg.send_json({"posx": None, "qr": None}, check=True).body
        if body["qr"] == tg.planner_buffers:
            break                                   # probe finished; the model position is reported now
        low = body["posx"] if low is None else min(low, body["posx"])
    assert low is not None, "never sampled the probe move"
    _run_until_stopped(tg)
    return low


def _hold_and_resume(tg, after):
    time.sleep(after)
    tg.send_raw(b"!")
    _wait_hold(tg)
    tg.send_raw(b"~")


@pytest.fixture
def start_x(tg):
    start = tg.get("posx")
    yield start
    if tg.get("hold") == tg.dialect.hold("HOLD"):
        tg.send_raw(b"~")
    tg.wait_idle()
    tg.send_gcode(f"G90 G1 X{start:.4f} F600", allow=MOVE)
    tg.wait_idle()
    tg.send_gcode("G91")


@pytest.mark.motion
@pytest.mark.resets_board
def test_resume_stops_at_endpoint(tg, start_x):
    tg.send_gcode("G91 G1 X4 F120", check=True, allow=MOVE)
    _hold_and_resume(tg, 0.8)
    peak = _run_until_stopped(tg)
    assert peak <= start_x + 4 + TOL, f"resumed move overshot to {peak:.3f} (endpoint {start_x + 4:.3f})"
    assert _runtime_end(tg) == pytest.approx(start_x + 4, abs=TOL)


@pytest.mark.motion
@pytest.mark.resets_board
def test_repeated_holds_on_one_move(tg, start_x):
    tg.send_gcode("G91 G1 X4 F120", check=True, allow=MOVE)
    _hold_and_resume(tg, 0.5)
    _hold_and_resume(tg, 0.5)
    _hold_and_resume(tg, 0.5)
    peak = _run_until_stopped(tg)
    assert peak <= start_x + 4 + TOL, f"overshot to {peak:.3f}"
    assert _runtime_end(tg) == pytest.approx(start_x + 4, abs=TOL)


@pytest.mark.motion
@pytest.mark.resets_board
def test_resume_with_queued_move(tg, start_x):
    """Hold in the first of two queued moves; the runtime must end where both were commanded."""
    tg.send_gcode("G91 G1 X4 F120", check=True, allow=MOVE)
    tg.send_gcode("G1 X2 F240", check=True, allow=MOVE)
    _hold_and_resume(tg, 0.8)
    peak = _run_until_stopped(tg)
    assert peak <= start_x + 6 + TOL, f"overshot to {peak:.3f}"
    assert _runtime_end(tg) == pytest.approx(start_x + 6, abs=TOL)


@pytest.mark.motion
@pytest.mark.resets_board
def test_hold_decelerating_across_short_moves(tg, start_x):
    """Fast short moves, so the deceleration spans block boundaries (DECEL_CONTINUE)."""
    for _ in range(10):
        tg.send_gcode("G91 G1 X0.4 F900", check=True, allow=MOVE)
    _hold_and_resume(tg, 0.12)
    peak = _run_until_stopped(tg)
    assert peak <= start_x + 4 + TOL, f"overshot to {peak:.3f}"
    assert _runtime_end(tg) == pytest.approx(start_x + 4, abs=TOL)
