"""Runtime session family "rt": idle heartbeat and quiescence token (.analysis/heartbeat-design.md, Phase A).

JSON only, RAM only, off after every reset:
  {"rt":null}                 hb (s, 0 off), qt (token or null), up (s since boot)
  {"rt":{"hb":1}}             heartbeat every 1-60 s while idle: {"hb":{"stat":..,"hold":..,"up":..,"qt":..}}
  {"rt":{"qt":"<32 hex>"}}    quiescence token, accepted only while nothing is moving, held or queued; cleared at
                              reset and by any state-changing input (G-code, settings writes, clear, flush, job kill,
                              motion start); reads, ENQ, '!' and '~' leave it.
No text ($rthb) or flat ({"rthb":..}) access, no mixing rt with other keys, and a token write stands alone.
A heartbeat is not a response: it never carries r or f and does not disturb a line-mode sender.
"""
import json
import time

import pytest

from tinyg_hil import sender

pytestmark = pytest.mark.dialect("d449")

TOKEN = "0123456789abcdef0123456789ABCDEF"
TOKEN2 = "fedcba9876543210fedcba9876543210"


def rt(tg, body, timeout=2.0):
    return tg.send_line(json.dumps({"rt": body}), timeout=timeout)


def collect(tg, seconds):
    end, out = time.monotonic() + seconds, []
    while time.monotonic() < end:
        ev = tg.transport.next_event(end - time.monotonic())
        if ev is not None:
            out.append(ev)
    return out


@pytest.fixture
def hb_off(tg):
    yield
    rt(tg, {"hb": 0})
    tg.drain(quiet=0.2)


@pytest.mark.resets_board
def test_rt_is_off_after_reset(tg):
    rt(tg, {"hb": 1})
    tg.reset()
    resp = rt(tg, None)
    assert resp.status == 0, resp.raw
    body = resp.body["rt"]
    assert body["hb"] == 0 and body["qt"] is None and isinstance(body["up"], int), resp.raw


def test_idle_heartbeats(tg, hb_off):
    resp = rt(tg, {"hb": 1})
    assert resp.status == 0 and resp.body["rt"]["hb"] == 1, resp.raw
    got = [e for e in collect(tg, 3.6) if e.kind == "heartbeat"]
    assert 2 <= len(got) <= 4, [e.raw for e in got]
    for e in got:
        hb = e.data["hb"]
        assert set(hb) == {"stat", "hold", "up", "qt"} and "r" not in e.data and "f" not in e.data, e.raw
    ups = [e.data["hb"]["up"] for e in got]
    assert ups == sorted(ups)
    rt(tg, {"hb": 0})
    tg.drain(quiet=0.2)
    assert not [e for e in collect(tg, 2.5) if e.kind == "heartbeat"]


@pytest.mark.motion
def test_no_heartbeat_while_moving(tg, hb_off):
    tg.send_gcode("G90", check=True)
    tg.send_gcode("G0 X0", check=True, allow="known start")
    tg.wait_idle()
    rt(tg, {"hb": 1})
    try:
        tg.send_gcode("G1 X10 F200", check=True, allow="3 s move")
        events = collect(tg, 2.0)                       # inside the move
        assert not [e for e in events if e.kind == "heartbeat"], [e.raw for e in events]
        tg.wait_idle()
        assert [e for e in collect(tg, 2.6) if e.kind == "heartbeat"], "no heartbeat once idle again"
    finally:
        tg.send_gcode("G0 X0", allow="return to start")
        tg.wait_idle()


def test_quiescence_token(tg, keep):
    keep("qv")
    resp = rt(tg, {"qt": TOKEN})
    assert resp.status == 0 and resp.body["rt"]["qt"] == TOKEN, resp.raw
    for read in ('{"sr":null}', '{"fb":null}', '{"rt":null}'):
        tg.send_line(read)
    tg.send_raw(b"\x05")                                # ENQ
    tg.send_raw(b"!")
    tg.send_raw(b"~")
    tg.drain(quiet=0.3)
    assert rt(tg, {"qt": None}).body["rt"]["qt"] == TOKEN, "a read or realtime control cleared the token"

    tg.send_gcode("G21", check=True)                    # any G-code line
    assert rt(tg, {"qt": None}).body["rt"]["qt"] is None

    assert rt(tg, {"qt": TOKEN2}).status == 0
    tg.set("qv", tg.get("qv"), allow="a settings write")
    assert rt(tg, {"qt": None}).body["rt"]["qt"] is None

    assert rt(tg, {"qt": TOKEN}).status == 0
    tg.send_json({"clear": None})                       # an action, though it looks like a read
    assert rt(tg, {"qt": None}).body["rt"]["qt"] is None


@pytest.mark.motion
def test_token_refused_while_moving(tg):
    tg.send_gcode("G90", check=True)
    tg.send_gcode("G0 X0", check=True, allow="known start")
    tg.wait_idle()
    try:
        tg.send_gcode("G1 X5 F300", check=True, allow="move under way")
        resp = rt(tg, {"qt": TOKEN})
        assert resp.status != 0, resp.raw
        tg.wait_idle()
        assert rt(tg, {"qt": None}).body["rt"]["qt"] is None
    finally:
        tg.send_gcode("G0 X0", allow="return to start")
        tg.wait_idle()


@pytest.mark.parametrize("line, why", [
    ('{"rthb":1}', "flat alias"),
    ('{"rt":{"hb":1},"sv":1}', "rt mixed with another key"),
    ('{"rt":{"hb":1,"qt":"' + TOKEN + '"}}', "token write with another operation"),
    ('{"rt":{"hb":61}}', "interval out of range"),
    ('{"rt":{"qt":"0123"}}', "token too short"),
    ('{"rt":{"qt":"0123456789abcdef0123456789abcdeg"}}', "token not hex"),
    ('{"rt":{"zz":1}}', "unknown leaf"),
])
def test_rt_rejects(tg, hb_off, line, why):
    resp = tg.send_line(line)
    assert resp.status != 0, f"{why}: {resp.raw}"
    body = rt(tg, None).body["rt"]
    assert body["hb"] == 0 and body["qt"] is None, f"{why} changed runtime state: {body}"


def test_rt_has_no_text_form(tg):
    lines = tg.send_text("$rthb=1")
    body = rt(tg, None).body["rt"]
    assert body["hb"] == 0, (lines, body)


@pytest.mark.motion
@pytest.mark.streaming
def test_heartbeat_does_not_disturb_line_sender(tg, keep, hb_off):
    keep("ej", "jv", "rxm", "ex", "qv")
    for token, value in {"ej": 1, "qv": 1, **sender.MODES["line"]}.items():
        if tg.get(token) != value:
            tg.set(token, value, allow="line-mode stream")
    rt(tg, {"hb": 1})
    tg.send_gcode("G90", check=True)
    tg.send_gcode("G0 X0", check=True, allow="known start")
    tg.wait_idle()
    lines = ["G91", "G1 F600"] + [f"G1 X{0.5 if i % 2 == 0 else -0.5}" for i in range(40)] + ["G90"]
    for line in lines:
        tg.guard.check_gcode(line, allow="short zigzag stream")
    res = sender.stream(tg, lines, "line", timeout=120)
    assert res.acked == res.lines
    tg.wait_idle()
    assert [e for e in collect(tg, 2.6) if e.kind == "heartbeat"], "no heartbeat after the stream"
