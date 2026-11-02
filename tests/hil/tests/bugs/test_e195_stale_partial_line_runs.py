"""E195: a partial line left by a host that went away runs when the next host connects.

A host that disconnects mid-line leaves the start of a line in the RX buffer. The next host's first line
({sr:null}) is appended to it, and the merged line runs when its LF arrives. If the leftover ends inside a
comment ('(' or ';'), everything appended is comment and the leftover runs as it stands: "G0 X0.5 (rough"
followed by {sr:null} moves X. No host prefix avoids it (diagnostics/test_leftover_line_reconnect.py
measured it on 440.21 and edge-next d254eb0b). The reply the new host gets is a G-code reply, not its sr.

Fix (maintainer decision 2026-11-02): a '{' that arrives after the line already in progress has been quiet
for more than a second starts a fresh line; the stale start is dropped. Measured from the last byte received,
so a slow host is never cut, and only '{' (how hosts open a session) triggers it.

Controls run on every build: the same bytes without the quiet gap stay one line (the old behaviour), and
a JSON line with a nested object sent in pieces with short pauses still answers normally.
"""
import time

import pytest

pytestmark = pytest.mark.motion

QUIET = 1.5         # s: longer than the firmware's stale-line time
SHORT = 0.3         # s: well inside it


def _at_x0(tg):
    tg.send_gcode("G90", check=True)
    tg.send_gcode("G0 X0", check=True, allow="known start")
    tg.wait_idle()
    tg.drain(quiet=0.1)
    return tg.get("posx")


def _replies(tg):
    return [e for e in tg.drain(quiet=0.5, limit=4.0) if e.kind in ("response", "exception")]


@pytest.mark.bug("E195", on=["d440"])
@pytest.mark.parametrize("leftover", ["G0 X0.5 (rough", "G0 X0.5 ;rough"], ids=["paren", "semicolon"])
def test_stale_comment_line_does_not_run_on_connect(tg, record, leftover):
    tg.guard.check_gcode(leftover, allow="leftover line on an unloaded bench")
    start = _at_x0(tg)
    try:
        tg.transport.write(leftover.encode())
        time.sleep(QUIET)
        tg.transport.write(b"{sr:null}\n")
        replies = _replies(tg)
        tg.wait_idle()
        end = tg.get("posx")
        record("connect", {"leftover": leftover, "replies": [e.raw for e in replies], "posx": [start, end]})
        assert end == pytest.approx(start, abs=0.01), f"the stale line ran: X {start} -> {end}"
        assert replies and "sr" in (replies[0].body or {}), f"first reply is not the sr: {[e.raw for e in replies]}"
    finally:
        tg.send_gcode("G0 X0", allow="return to start")
        tg.wait_idle()


def test_line_without_quiet_gap_stays_one_line(tg, record):
    """No gap: the leftover and the appended text are one line, as before (the comment hides the JSON)."""
    left = "G0 X0.5 (rough"
    tg.guard.check_gcode(left, allow="leftover line on an unloaded bench")
    start = _at_x0(tg)
    try:
        tg.transport.write(left.encode())
        time.sleep(SHORT)
        tg.transport.write(b"{sr:null}\n")
        replies = _replies(tg)
        tg.wait_idle()
        end = tg.get("posx")
        record("no-gap", {"replies": [e.raw for e in replies], "posx": [start, end]})
        assert end == pytest.approx(0.5, abs=0.01)
    finally:
        tg.send_gcode("G0 X0", allow="return to start")
        tg.wait_idle()


def test_slow_line_with_brace_inside(tg):
    """A host that sends a line in pieces with short pauses, one of them just before a '{' inside the line."""
    tg.drain(quiet=0.1)
    for part in (b'{"gc":"G4 P0 (', b'{note})"}\n'):
        tg.transport.write(part)
        time.sleep(SHORT)
    replies = _replies(tg)
    assert replies and replies[0].kind == "response" and replies[0].data["f"][1] == 0, [e.raw for e in replies]
