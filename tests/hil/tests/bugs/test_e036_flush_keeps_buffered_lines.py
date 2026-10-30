"""E036: '!' then '%' leaves lines already in the line-mode buffers; they run once the flush ends the hold.

edge canonical_machine.c:1566-1573 cm_request_queue_flush() only calls xio_flush_read(), which is
xio_reset_usb_rx_buffers (xio.h:100, xio_usb.c:203-212) and resets the RX ring. The line-mode header pool
(xio.c:378-399, RX_HEADERS 12) is never cleared: xio_reset_readline_linemode() (xio.c:407) has no callers.
_dispatch_control (controller.c:157) keeps reading DATA lines into the pool while _sync_to_planner (:175)
blocks them, and cm_queue_flush (:1679-1693) flushes the planner then cm_end_hold(), so the next
_dispatch_command runs the stale lines without '~'. The doc at :1666 promises to reject them.
"""
import json
import time

import pytest

MOVE = "up to 3.6 mm of 0.1 mm +X moves at F30, flushed, then back"
LINE = "G1 X0.1 F30"         # 0.2 s each


def _query(tg, *tokens, timeout=2.0):
    """GET in one line, skipping responses to lines that were written without waiting."""
    tg.transport.write(json.dumps({t: None for t in tokens}, separators=(",", ":")).encode() + b"\n")
    ev = tg.wait_for(lambda e: e.kind == "response" and isinstance(e.body, dict)
                     and all(t in e.body for t in tokens), timeout)
    return ev.body


@pytest.mark.motion
@pytest.mark.persists
@pytest.mark.dialect("d449")
@pytest.mark.bug("E036", on=["d449"])
def test_flush_drops_buffered_lines(tg, keep):
    keep("rxm")
    tg.set("rxm", 1)
    start = tg.get("mpox")
    tg.send_gcode("G91", check=True)
    for _ in range(28):
        tg.send_gcode(LINE, check=True, allow=MOVE)
    tg.transport.write((LINE + "\n").encode() * 8)     # these wait for planner buffers in the line pool
    try:
        time.sleep(0.1)
        tg.send_raw(b"!")
        end = time.monotonic() + 3
        while _query(tg, "hold", "vel") != {"hold": tg.dialect.hold("HOLD"), "vel": 0}:
            assert time.monotonic() < end, "feedhold did not reach HOLD"
        tg.send_raw(b"%")
        time.sleep(0.5)
        samples = [_query(tg, "vel", "qr") for _ in range(60)]      # ~1.5 s
        assert all(s["vel"] == 0 for s in samples), "motion resumed after the flush"
        assert samples[-1]["qr"] == tg.planner_buffers
    finally:
        end = time.monotonic() + 15
        while _query(tg, "qr")["qr"] != tg.planner_buffers and time.monotonic() < end:
            time.sleep(0.1)
        tg.drain(quiet=0.3)
        back = start - tg.get("mpox")
        if abs(back) > 0.0005:
            tg.send_gcode(f"G91 G1 X{back:.4f} F600", allow=MOVE)
            tg.wait_idle()
