"""Feed-limit report of the move stepping now (rt leaves flk, flm, flp, flf; feedlimit.h, runtime.c).

  flk  kind: 0 no move running (idle, held, dwelling), 1 feed move, 2 rapid
  flm  constraint tags of the running move's plan (FL_* bits)
  flp  planned cruise as a percent of the move's F (G94 linear XYZ feed moves; else null)
  flf  the move's F in mm/min (null when no move runs)

Checks on the board: idle reads 0/0/null/null; a long feed move within fr reads kind 1, its F and about
100 %; the same move above X's feedrate max is tagged as axis-capped with the percent falling to
fr/F; a rapid reads kind 2 with no percent and no F; a dense zigzag reads too-short or single-segment tags;
a dwell and a feedhold read kind 0; two moves with a dwell between are
reported in turn; the leaves are read-only.
"""
import json
import time

import pytest

pytestmark = [pytest.mark.dialect("d449"), pytest.mark.motion]

FL_AXIS_CAP, FL_SHORT, FL_MIN_SEGMENT = 0x01, 0x08, 0x10


def fl(tg):
    resp = tg.send_line(json.dumps({"rt": {"flk": None, "flm": None, "flp": None, "flf": None}}))
    assert resp.status == 0, resp.raw
    return resp.body["rt"]


@pytest.fixture
def at_x0(tg):
    tg.send_gcode("G90 G21", check=True)
    tg.send_gcode("G0 X0 Y0", check=True, allow="known start")
    tg.wait_idle()
    yield
    tg.send_raw(b"~")
    tg.wait_idle()
    tg.send_gcode("G0 X0 Y0", allow="return to start")
    tg.wait_idle()


def test_idle(tg):
    tg.wait_idle()
    assert fl(tg) == {"flk": 0, "flm": 0, "flp": None, "flf": None}


def test_feed_move_within_fr(tg, at_x0):
    tg.send_gcode("G1 X20 F600", check=True, allow="2 s feed move")
    time.sleep(0.8)
    r = fl(tg)
    assert r["flk"] == 1 and r["flf"] == pytest.approx(600) and not (r["flm"] & FL_AXIS_CAP), r
    assert 95 <= r["flp"] <= 101, r
    tg.wait_idle()


def test_axis_cap(tg, at_x0):
    xfr = tg.get("xfr")
    f = round(xfr * 1.5)
    tg.send_gcode(f"G1 X40 F{f}", check=True, allow="feed above X's feedrate max")
    time.sleep(0.8)
    r = fl(tg)
    assert r["flk"] == 1 and (r["flm"] & FL_AXIS_CAP), r
    assert r["flp"] == pytest.approx(100 * xfr / f, abs=2), r
    tg.wait_idle()


def test_rapid(tg, at_x0):
    tg.send_gcode("G0 X40", check=True, allow="rapid")
    time.sleep(0.15)
    r = fl(tg)
    if r["flk"] != 0:                                    # still running when sampled
        assert r["flk"] == 2 and r["flp"] is None and r["flf"] is None, r
    tg.wait_idle()


def test_dense_zigzag_is_short(tg, at_x0):
    lines = ["G91"] + [f"G1 X{0.3 if i % 2 == 0 else -0.3} F1500" for i in range(30)] + ["G90"]
    for line in lines:
        tg.guard.check_gcode(line, allow="zigzag")
        tg.send_gcode(line, check=True, allow="zigzag")
    seen = 0
    for _ in range(6):
        r = fl(tg)
        if r["flk"] == 1:
            seen |= r["flm"]
        time.sleep(0.05)
    tg.wait_idle()
    assert seen & (FL_SHORT | FL_MIN_SEGMENT), f"zigzag moves never reported short: {seen:#x}"


def test_dwell_and_hold_read_no_move(tg, at_x0):
    tg.send_gcode("G4 P1.5", check=True)
    time.sleep(0.5)
    assert fl(tg)["flk"] == 0
    tg.wait_idle()
    tg.send_gcode("G1 X30 F600", check=True, allow="move to hold")
    time.sleep(0.6)
    tg.send_raw(b"!")
    time.sleep(1.0)
    assert fl(tg)["flk"] == 0
    tg.send_raw(b"~")
    time.sleep(0.4)
    assert fl(tg)["flk"] == 1
    tg.wait_idle()


def test_consecutive_moves_with_a_dwell_between(tg, at_x0):
    """The report follows the real exec, prep and load chain from one move to the next."""
    tg.send_gcode("G1 X10 F500", check=True, allow="first move")
    tg.send_gcode("G4 P1.2", check=True)
    tg.send_gcode("G1 X20 F900", check=True, allow="second move")
    time.sleep(0.5)
    first = fl(tg)
    assert first["flk"] == 1 and first["flf"] == pytest.approx(500), first
    deadline = time.monotonic() + 6
    seen_dwell = seen_second = False
    while time.monotonic() < deadline and not seen_second:
        r = fl(tg)
        seen_dwell = seen_dwell or r["flk"] == 0
        seen_second = r["flk"] == 1 and r["flf"] == pytest.approx(900)
        time.sleep(0.1)
    assert seen_dwell and seen_second, (seen_dwell, seen_second)
    tg.wait_idle()


def test_leaves_read_only(tg):
    for leaf in ("flk", "flm", "flp", "flf"):
        resp = tg.send_line(json.dumps({"rt": {leaf: 1}}))
        assert resp.status != 0, f"{leaf}: {resp.raw}"
