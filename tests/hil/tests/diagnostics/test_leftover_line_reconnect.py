"""Diagnostic: a partial G-code line left in the RX buffer by one host, then a new host's first line.

A host that disconnects mid-line leaves the start of a G-code line in the board's RX buffer. The next
host's first line is appended to it, and the merged line runs when that line's LF arrives. The question
is what the merged line does, for the status request a connecting host sends first ({sr:null} with a
bare key, or {"sr":null}):
  - 440.21 (controller.c, default branch of _dispatch_kernel) wraps a G-code line in JSON mode as
    {"gc":"<line>"}. A '"' in the appended text ends the gc string early, so "G0 X0.5" + {"sr":null}
    can run as a truncated G0 X0.5. With the bare key the line normalizes to G0X0.5SRNULL, and S
    followed by RNULL fails the number check (STAT_BAD_NUMBER_FORMAT).
  - edge-next sends the line straight to gc_gcode_parser(); _normalize_gcode_block() drops '{', '"'
    and ':', so both forms become ...SRNULL and are rejected.
  - On both, a leftover that ends inside a comment ('(' or ';') makes everything after it comment,
    so the leftover runs as it stands, whatever the new host sends.

Each case starts idle at a known X, writes the leftover without a line end, waits as a reconnect
would, sends the connect line, and records every reply and whether X moved. The table is the record;
the test fails only when the board ends a case outside the expected states (alarm, shutdown, panic).
"""
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion]

LEFTOVERS = {
    "move": "G0 X0.5",
    "mid-number": "G0 X0.",
    "paren-comment": "G0 X0.5 (rough",
    "semicolon-comment": "G0 X0.5 ;rough",
}
CONNECTS = {"bare": "{sr:null}", "quoted": '{"sr":null}'}
GAP = 0.5           # s between the old host's last byte and the new host's first


@pytest.mark.parametrize("connect", list(CONNECTS), ids=list(CONNECTS))
@pytest.mark.parametrize("leftover", list(LEFTOVERS), ids=list(LEFTOVERS))
def test_leftover_line_then_connect(tg, record, leftover, connect):
    left, first = LEFTOVERS[leftover], CONNECTS[connect]
    tg.guard.check_gcode(left, allow="leftover line on an unloaded bench")
    tg.send_gcode("G90", check=True)
    tg.send_gcode("G0 X0", check=True, allow="known start")
    tg.wait_idle()
    start = tg.get("posx")
    tg.drain(quiet=0.1)
    try:
        tg.transport.write(left.encode())
        time.sleep(GAP)
        tg.transport.write(first.encode() + b"\n")
        events = tg.drain(quiet=0.5, limit=4.0)
        tg.wait_idle()
        end = tg.get("posx")
        moved = abs(end - start) > 0.01
        replies = [e.raw for e in events if e.kind in ("response", "exception")]
        record("case", {"build": tg.identity["fb"], "leftover": left, "connect": first, "moved": moved,
                        "posx": [start, end], "replies": replies,
                        "statuses": [e.data["f"][1] for e in events if e.kind == "response"]})
        print(f"{tg.identity['fb']} {leftover:18} {connect:6} moved={moved} replies={replies}")
        assert tg.stat() not in tg.dialect.stats("ALARM", "SHUTDOWN", "PANIC")
    finally:
        tg.send_gcode("G0 X0", allow="return to start")
        tg.wait_idle()
