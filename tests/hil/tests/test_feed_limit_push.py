"""Feed-limit push (step 2): {rt:{fi:N}} opens an accounting window; the board pushes fs summaries and fl
cause-change lines (runtime.c, feedlimit.c, stepper.c; .analysis/feed-advisor-design.md).

  {"fs":{"v":2,"e":window,"q":seq,"c":[ms per cause...],"ct":cutting_ms,"cd":cutting_mm}}
      every N s while a cycle runs, and once more after the cycle has drained; cumulative since the window
      opened. c order: axis cap, corner, exact stop, short, single segment, other, command boundary (v2).
      Overlapping counters.
  {"fl":{"k":kind,"m":tags,"p":pct,"n":line}}
      when the executing move's kind or tags change, at most 4 a second; k 0 when nothing runs.

Checks on the board: off by default and silent; N clamps to 60 and the echo says so; a window opens only at
quiescence; a job's summaries count up from q 1 in one window, the final one carries the commanded cutting
distance and a cutting time no shorter than the moves at cruise, the axis-capped move's time under the axis
cause and corner time under the corner cause; fl lines name the executing lines and end with k 0; rapids are
not cutting; a feedhold's deceleration is not counted; {rt:{fs:null}} pushes a summary with the q it returns.
"""
import json
import time

import pytest

pytestmark = [pytest.mark.dialect("d449"), pytest.mark.motion]

AXIS_CAP, CORNER = 0, 1                                  # indexes into fs "c"


def rt(tg, body, timeout=2.0):
    return tg.send_line(json.dumps({"rt": body}), timeout=timeout)


def since(tg, mark, kind):
    return [e.data[{"feedlimit": "fl", "feedsummary": "fs", "feedgaps": "fg"}[kind]] for e in tg.transport.history[mark:] if e.kind == kind]


def settle(tg, seconds):                                 # let pushed lines arrive (they are kept in history)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        tg.transport.next_event(end - time.monotonic())


@pytest.fixture
def fi_off(tg):
    rt(tg, {"fi": 0})
    yield
    rt(tg, {"fi": 0})


@pytest.fixture
def at_x0(tg, fi_off):
    tg.send_gcode("G90 G21", check=True)
    tg.send_gcode("G0 X0 Y0", check=True, allow="known start")
    tg.wait_idle()
    yield
    tg.send_raw(b"~")
    tg.wait_idle()
    rt(tg, {"fi": 0})
    tg.send_gcode("G90 G0 X0 Y0", allow="return to start")
    tg.wait_idle()


def test_off_by_default_is_silent(tg, at_x0):
    mark = len(tg.transport.history)
    tg.send_gcode("G1 X10 F600", check=True, allow="1 s feed move")
    tg.wait_idle()
    settle(tg, 1.2)
    assert not since(tg, mark, "feedlimit") and not since(tg, mark, "feedsummary")


def test_interval_clamps_and_echoes(tg, fi_off):
    resp = rt(tg, {"fi": 200})
    assert resp.status == 0 and resp.body["rt"]["fi"] == 60, resp.raw
    resp = rt(tg, {"fi": 2})
    assert resp.status == 0 and resp.body["rt"]["fi"] == 2, resp.raw
    assert rt(tg, {"fi": None}).body["rt"]["fi"] == 2


def test_window_opens_only_at_quiescence(tg, at_x0):
    tg.send_gcode("G1 X20 F400", check=True, allow="3 s feed move")
    time.sleep(0.4)
    resp = rt(tg, {"fi": 1})
    assert resp.status == tg.dialect.status("COMMAND_NOT_ACCEPTED"), resp.raw
    tg.wait_idle()
    assert rt(tg, {"fi": None}).body["rt"]["fi"] == 0


def test_job_summaries_and_events(tg, at_x0):
    xfr = tg.get("xfr")
    f_cap = round(xfr * 1.5)
    assert rt(tg, {"fi": 1}).status == 0
    mark = len(tg.transport.history)
    lines = [f"N10 G1 X30 F{f_cap}", "N20 G1 Y10 F600", "N30 G1 X0 F600", "N40 G1 Y0 F600"]
    for line in lines:
        tg.send_gcode(line, check=True, allow="80 mm feed square, X above its fr")
    tg.wait_idle(timeout=60)
    settle(tg, 1.5)
    fs = since(tg, mark, "feedsummary")
    assert fs, "no summary"
    assert [s["q"] for s in fs] == list(range(1, len(fs) + 1)) and len({s["e"] for s in fs}) == 1, fs
    final = fs[-1]
    assert all(s["v"] == 2 and len(s["c"]) == 7 for s in fs)
    assert final["cd"] == pytest.approx(80.0, abs=0.4), final
    at_cruise_ms = 60000 * (30 / xfr + 50 / 600)
    assert at_cruise_ms <= final["ct"] <= 1.6 * at_cruise_ms, (final, at_cruise_ms)
    assert final["c"][AXIS_CAP] >= 0.9 * 60000 * 30 / xfr, final
    assert final["c"][CORNER] > 0, final
    fl = since(tg, mark, "feedlimit")
    assert fl and fl[-1]["k"] == 0 and fl[-1]["n"] is None, fl
    assert {e["n"] for e in fl if e["k"]} <= {10, 20, 30, 40}, fl
    assert any(e["k"] == 1 and e["n"] == 10 and (e["m"] & 1) for e in fl), fl


def test_rapids_are_not_cutting(tg, at_x0):
    assert rt(tg, {"fi": 1}).status == 0
    mark = len(tg.transport.history)
    tg.send_gcode("G0 X30", check=True, allow="rapid")
    tg.wait_idle()
    settle(tg, 1.5)
    final = since(tg, mark, "feedsummary")[-1]
    assert final["ct"] == 0 and final["cd"] == 0, final


def test_hold_deceleration_is_not_counted(tg, at_x0):
    assert rt(tg, {"fi": 1}).status == 0
    mark = len(tg.transport.history)
    tg.send_gcode("G1 X40 F1200", check=True, allow="2 s feed move, held and resumed")
    time.sleep(0.9)
    tg.send_raw(b"!")
    time.sleep(1.0)
    tg.send_raw(b"~")
    tg.wait_idle(timeout=20)
    settle(tg, 1.5)
    final = since(tg, mark, "feedsummary")[-1]
    assert 30.0 < final["cd"] < 39.95, final                # the deceleration into the hold is left out


def test_summary_on_request(tg, at_x0):
    assert rt(tg, {"fi": 1}).status == 0
    tg.send_gcode("G1 X5 F600", check=True, allow="short feed move")
    tg.wait_idle()
    settle(tg, 1.2)
    mark = len(tg.transport.history)
    resp = rt(tg, {"fs": None})
    assert resp.status == 0 and isinstance(resp.body["rt"]["fs"], int), resp.raw
    settle(tg, 0.5)
    fs = since(tg, mark, "feedsummary")
    assert fs and fs[-1]["q"] == resp.body["rt"]["fs"] and fs[-1]["cd"] == pytest.approx(5.0, abs=0.05), fs
