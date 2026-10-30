"""E094: the line-mode RX allocator ignores the wrapped used region, so new input overwrites queued
lines that have not run yet once lines average more than about 66 characters.

xio.c:461-476 (_get_free_buffer) places the next buffer just past used_top and only checks the room
up to pool_top, never whether used_base lies in between after the wrap at xio.c:478-481; the fill
at xio.c:668/682 writes up to 200 bytes regardless. Pool 1000 B, 12 headers (xio.h:214-216).
Line mode is the 449 default; 440 has no line mode.
This is a board stress check, not a reliable stock reproducer: the staged 40/85-byte
stream passed on stock. tests/host/test_linemode_allocator.py deterministically
reproduces the allocator overlap using the actual firmware functions.
"""
import time

import pytest

from tinyg_hil.protocol import XOFF

pytestmark = pytest.mark.dialect("d449")
LINES = 60


def _stream(tg, data: bytes, chunk=16, gap=0.01, limit=60.0):
    """Write in small, spaced chunks and pause while the board's last flow byte is XOFF.

    The board sends XOFF with 50 of its 250 RX bytes left, so the pacing keeps what is in flight
    during the host's reaction time below that.
    """
    end = time.monotonic() + limit
    for i in range(0, len(data), chunk):
        while tg.transport.flow and tg.transport.flow[-1][1] == XOFF:
            assert time.monotonic() < end, "board stayed in XOFF"
            time.sleep(0.002)
        tg.transport.write(data[i:i + chunk])
        time.sleep(gap)


def _line(k: int, length: int) -> str:
    head = f"N{k} G4 P0.15 ("
    return head + f"{k}".rjust(length - len(head) - 1, "A") + ")"


@pytest.mark.resets_board
@pytest.mark.parametrize("length", [
    pytest.param(72, id="72-chars"),
    pytest.param(85, id="85-chars"),
    pytest.param(40, id="40-chars"),
])
@pytest.mark.timeout(180)
def test_streamed_lines_run_in_order(tg, length):
    if tg.get("rxm") != 1:
        pytest.skip("requires line mode ($rxm=1)")
    if tg.get("ex") != 1:
        pytest.skip("the streamer relies on XON/XOFF flow control ($ex=1)")
    try:
        tg.send_gcode("G4 P2", check=True)      # holds the planner so the RX pool fills up
        _stream(tg, "".join(_line(k, length) + "\n" for k in range(1, LINES + 1)).encode())
        ns, statuses, end = [], [], time.monotonic() + 60
        while LINES not in ns:
            ev = tg.transport.next_event(end - time.monotonic())
            if ev is None:
                break
            if ev.kind == "response" and "n" in ev.body:
                ns.append(ev.body["n"])
                statuses.append(ev.data["f"][1])
        assert ns == list(range(1, LINES + 1)), ns
        assert set(statuses) == {0}
    finally:
        tg.reset()
