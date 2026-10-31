"""E145: an alarm's queue flush drops host input when motion comes to rest, cutting a line in flight.

cm_alarm() requests a queue flush; since E036 _run_requested_queue_flush() (canonical_machine.c) calls
xio_flush_input() once the runtime is idle, which is seconds after the alarm. Whatever the host has
sent by then is discarded unanswered, and a line half received at that moment loses its head: on
bench 1 a getter came back as rejected G-code (204) or as a lone '}' (ICE readback of cs.saved_buf).
Here the head of a line is sent right after the alarm and its tail after the stop has finished.
"""
import time

import pytest

pytestmark = [pytest.mark.dialect("d449"), pytest.mark.motion]

MOVE = "5 mm X move stopped by an alarm, then back"


@pytest.mark.bug("E145", on=["d449"])
def test_line_split_across_alarm_stop_is_answered(tg):
    tg.send_gcode("G21 G91", check=True)
    try:
        tg.send_gcode("G1 X5 F100", check=True, allow=MOVE)
        time.sleep(0.5)
        tg.send_json({"alarm": None})
        tg.transport.write(b'{"stat":')          # head now; the stop is still decelerating
        time.sleep(3.0)                          # the stop has finished: the old code flushed input here
        tg.transport.write(b'null}\n')
        ev = tg.wait_for(lambda e: e.kind == "response", 2.0)
        assert ev is not None, "no reply to the split line"
        assert ev.data["f"][1] == 0 and "stat" in ev.body, ev.raw
        assert ev.body["stat"] == tg.dialect.stat("ALARM")
    finally:
        tg.drain()
        tg.clear_alarm()
        tg.send_gcode("G1 X-5 F600", allow=MOVE)
        tg.wait_idle()
        tg.send_gcode("G90")
