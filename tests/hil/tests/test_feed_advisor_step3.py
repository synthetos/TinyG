"""Feed-limit advisor step 3 on the board: the host job flag, supply gaps, command boundaries and the opt-in fg
frame (runtime.c, feedlimit.c, stepper.c, planner.c; .analysis/feed-advisor-design.md).

  {"rt":{"job":1|0}}  a host streams a job with more motion to send; reset, %, job kill and M2/M30 clear it.
  {"rt":{"fg":1|0}}   follow each fs with {"fg":{"e","q","g":[ms_pending,ms_none,gaps],"b":[[n,ms] x3],"x"?}}
                      (b: spindle/S, coolant, other). Off by default: a host that does not know fg must never see it.
  fl while starved:   {"fl":{"k":0,"m":0,"p":null,"n":<last line run>,"g":1|2}}

A gap needs accounting on, job=1 and motion begun in the job, and the runtime fully drained; a hold pauses it.
Hosts send job 0 together with their last motion line, so the normal end of a job is never a gap.
"""
import json
import time

import pytest

pytestmark = [pytest.mark.dialect("d449"), pytest.mark.motion]


def rt(tg, body, timeout=2.0):
    return tg.send_line(json.dumps({"rt": body}), timeout=timeout)


def since(tg, mark, kind):
    key = {"feedlimit": "fl", "feedsummary": "fs", "feedgaps": "fg"}[kind]
    return [e.data[key] for e in tg.transport.history[mark:] if e.kind == kind]


def settle(tg, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        tg.transport.next_event(end - time.monotonic())


def pair(tg, mark):
    """The last fs and the fg with its e and q."""
    fs, fg = since(tg, mark, "feedsummary"), since(tg, mark, "feedgaps")
    assert fs and fg, (fs, fg)
    match = [g for g in fg if (g["e"], g["q"]) == (fs[-1]["e"], fs[-1]["q"])]
    assert match, (fs[-1], fg)
    return fs[-1], match[-1]


@pytest.fixture
def window(tg):
    rt(tg, {"fi": 0})
    rt(tg, {"job": 0})
    rt(tg, {"fg": 0})
    tg.send_gcode("G90 G21", check=True)
    tg.send_gcode("G0 X0 Y0", check=True, allow="known start")
    tg.wait_idle()
    assert rt(tg, {"fi": 1}).status == 0
    yield
    tg.send_raw(b"~")
    tg.wait_idle()
    rt(tg, {"job": 0})
    rt(tg, {"fg": 0})
    rt(tg, {"fi": 0})
    tg.send_gcode("G90 G0 X0 Y0", allow="return to start")
    tg.wait_idle()


def test_flags_read_write(tg, window):
    assert rt(tg, {"job": None}).body["rt"]["job"] == 0 and rt(tg, {"fg": None}).body["rt"]["fg"] == 0
    assert rt(tg, {"job": 2}).status == tg.dialect.status("INPUT_VALUE_RANGE_ERROR")
    assert rt(tg, {"job": 1}).status == 0 and rt(tg, {"job": None}).body["rt"]["job"] == 1
    assert rt(tg, {"fg": 1}).status == 0 and rt(tg, {"fg": None}).body["rt"]["fg"] == 1
    tg.send_gcode("M2", check=True)                                       # program end ends the job
    tg.wait_idle()
    assert rt(tg, {"job": None}).body["rt"]["job"] == 0
    assert rt(tg, {"fg": None}).body["rt"]["fg"] == 1                     # the opt-in stays


def test_fg_only_after_opt_in(tg, window):
    mark = len(tg.transport.history)
    tg.send_gcode("G1 X5 F600", check=True, allow="short feed move")
    tg.wait_idle()
    settle(tg, 1.2)
    rt(tg, {"fs": None})
    settle(tg, 0.5)
    assert since(tg, mark, "feedsummary") and not since(tg, mark, "feedgaps")
    assert rt(tg, {"fg": 1}).status == 0
    mark = len(tg.transport.history)
    q = rt(tg, {"fs": None}).body["rt"]["fs"]
    settle(tg, 0.5)
    fs, fg = pair(tg, mark)
    assert fs["q"] == q and fs["v"] == 2 and len(fs["c"]) == 7
    assert fg["g"] == [0, 0, 0] and fg["b"] == [[0, 0], [0, 0], [0, 0]] and "x" not in fg, fg


def test_starved_job_is_a_gap(tg, window):
    assert rt(tg, {"fg": 1}).status == 0 and rt(tg, {"job": 1}).status == 0
    mark = len(tg.transport.history)
    tg.send_gcode("N10 G1 X10 F600", check=True, allow="1 s feed move")
    tg.wait_idle()
    settle(tg, 1.5)                                                       # the host sends nothing: starved
    starved = [e for e in since(tg, mark, "feedlimit") if "g" in e]
    assert starved and starved[-1] == {"k": 0, "m": 0, "p": None, "n": 10, "g": 2}, since(tg, mark, "feedlimit")
    tg.send_gcode("N20 G1 X0 F600", check=True, allow="1 s feed move back")
    rt(tg, {"job": 0})                                                    # with the last motion line
    tg.wait_idle()
    settle(tg, 1.2)
    fs, fg = pair(tg, mark)
    # the next line can sit on the board for a few ms before it runs: that short interval is "input pending"
    assert fg["g"][2] == 1 and fg["g"][0] < 100 and 1200 <= fg["g"][1] <= 4000, fg
    assert fs["cd"] == pytest.approx(20.0, abs=0.2), fs


def test_no_gap_without_a_job(tg, window):
    assert rt(tg, {"fg": 1}).status == 0
    mark = len(tg.transport.history)
    tg.send_gcode("G1 X10 F600", check=True, allow="1 s feed move")
    tg.wait_idle()
    settle(tg, 1.5)
    tg.send_gcode("G1 X0 F600", check=True, allow="back")
    tg.wait_idle()
    settle(tg, 1.2)
    _, fg = pair(tg, mark)
    assert fg["g"] == [0, 0, 0], fg
    assert not [e for e in since(tg, mark, "feedlimit") if "g" in e]


def test_hold_is_not_a_gap(tg, window):
    assert rt(tg, {"fg": 1}).status == 0 and rt(tg, {"job": 1}).status == 0
    mark = len(tg.transport.history)
    tg.send_gcode("G1 X40 F1200", check=True, allow="2 s feed move, held and resumed")
    time.sleep(0.8)
    tg.send_raw(b"!")
    time.sleep(1.5)
    tg.send_raw(b"~")
    time.sleep(0.3)
    tg.send_gcode("G1 X0 F1200", check=True, allow="back, sent while still moving")
    rt(tg, {"job": 0})
    tg.wait_idle(timeout=20)
    settle(tg, 1.2)
    _, fg = pair(tg, mark)
    assert fg["g"] == [0, 0, 0], fg


def test_command_between_moves_is_a_break(tg, window):
    assert rt(tg, {"fg": 1}).status == 0 and rt(tg, {"job": 1}).status == 0
    mark = len(tg.transport.history)
    for line in ("G1 X10 F600", "S1000", "M8", "G1 X20 F600", "M9"):
        tg.send_gcode(line, check=True, allow="feed moves around an S and coolant words")
    tg.send_gcode("G1 X30 F600", check=True, allow="last move")
    # job writes take effect at once (an rt line overtakes buffered moves), so end the job only once M9 has
    # executed, during the last move: earlier, the queued commands would run after the job and not count
    end = time.monotonic() + 15
    while tg.get("posx") < 22 and time.monotonic() < end:
        time.sleep(0.1)
    rt(tg, {"job": 0})
    tg.wait_idle(timeout=20)
    settle(tg, 1.2)
    fs, fg = pair(tg, mark)
    assert fg["b"][0][0] == 1 and fg["b"][1][0] == 1, fg                 # S+M8 = one break (spindle); M9 = coolant
    assert fg["b"][0][1] > 0 and fs["c"][6] > 0, (fs, fg)
    assert fg["g"][2] == 0, fg
