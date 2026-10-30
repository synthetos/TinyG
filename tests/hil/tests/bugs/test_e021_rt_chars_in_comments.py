"""E021: '!', '~' and '%' are trapped by the USB RX ISR anywhere in the stream, even inside a comment.

xio/xio_usb.c:146-157 (edge) consumes the three characters before they reach the line buffer, so a
'%' in a comment or (MSG ...) is always stripped and a '!' in a comment feedholds a running job.
Master's xio_usb.c ISR traps them the same way.
"""
import time

import pytest


@pytest.mark.bug("E021", on=["d440"])
@pytest.mark.parametrize("message", ["50% done", "note!", "note~", "!~% together"])
def test_percent_in_msg_is_kept(tg, message):
    # Idle, so the stripped '%' has no side effect beyond vanishing from the text.
    assert tg.send_gcode(f"G21 (MSG {message})", check=True).body.get("msg", "").strip() == message


def test_msg_without_realtime_chars(tg):
    assert tg.send_gcode("G21 (MSG 50 done)", check=True).body.get("msg", "").strip() == "50 done"


@pytest.mark.motion
@pytest.mark.bug("E021", on=["d440"])
def test_bang_in_comment_does_not_feedhold(tg):
    seen = []
    tg.send_gcode("G91 G1 X5 F300", check=True, allow="5 mm relative move, returned below")
    try:
        tg.send_gcode("G4 P0.01 (note!)", check=True)
        end = time.monotonic() + 0.8      # the X5 move takes about 1 s
        while time.monotonic() < end:
            seen.append(tg.get("hold"))
    finally:
        if tg.get("hold") != 0:
            tg.send_raw(b"~")
        tg.wait_idle()
        tg.send_gcode("G91 G1 X-5 F300", check=True, allow="return to start")
        tg.wait_idle()
        tg.send_gcode("G90", check=True)
    assert seen and all(state == 0 for state in seen), seen
