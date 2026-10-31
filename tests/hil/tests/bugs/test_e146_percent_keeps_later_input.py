"""E146: a host '%' drops input the host sent after it, not only what came before.

E036 resets the whole RX ring, the line being read and the line pool when the main loop runs the
requested flush, which waits until a feedhold has finished decelerating. Bytes the host sends after
the '%' in that window are lost, and a line straddling the flush loses its head (test_e117's cleanup
saw a query cut). Here '!%' goes out during motion with the head of a query right behind it; the tail
follows after the flush.
"""
import time

import pytest

pytestmark = [pytest.mark.dialect("d449"), pytest.mark.motion]

MOVE = "20 mm +X at F600, held and flushed mid-move, then back"


@pytest.mark.bug("E146", on=["d449"])
def test_line_split_across_percent_flush_is_kept(tg):
    start = tg.get("posx")
    tg.send_gcode("G91 G1 X20 F600", check=True, allow=MOVE)
    try:
        time.sleep(0.5)
        tg.transport.write(b'!%{"f')                 # hold, flush, and a query's head while decelerating
        time.sleep(2.0)                              # the hold (about 0.6 s here) has finished and the flush has run
        tg.transport.write(b'b":null}\n')
        ev = tg.wait_for(lambda e: e.kind == "response", 2.0)
        assert ev is not None, "no reply to the query"
        assert ev.data["f"][1] == 0 and "fb" in ev.body, f"the query lost its head: {ev.raw}"
    finally:
        tg.drain()
        tg.wait_idle(timeout=10)
        tg.send_gcode(f"G91 G1 X{start - tg.get('posx'):.4f} F600", allow=MOVE)
        tg.wait_idle()
        tg.send_gcode("G90")
