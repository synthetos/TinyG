"""E021: resume/flush are line-boundary controls; feedhold is immediate anywhere.

The host ISR fixture covers literal comment/MSG/JSON bytes and every boundary
control without timing assumptions. Serial tests check text and pause outcomes;
they do not measure feedhold latency.
"""
import time

import pytest


@pytest.mark.bug("E021", on=["d440"])
@pytest.mark.parametrize("message", ["50% done", "note~", "~% together"])
def test_percent_in_msg_is_kept(tg, message):
    assert tg.send_gcode(f"G21 (MSG {message})", check=True).body.get("msg", "").strip() == message


@pytest.mark.bug("E021", on=["d440"])
def test_percent_and_resume_in_json_string_are_kept(tg):
    message = "50% ~ done"
    assert tg.send_json({"msg": message}, check=True).body.get("msg", "").strip() == message


def test_msg_without_realtime_chars(tg):
    assert tg.send_gcode("G21 (MSG 50 done)", check=True).body.get("msg", "").strip() == "50 done"


@pytest.mark.motion
def test_bang_mid_line_still_feedholds(tg):
    """Feedhold within a partial comment is intentional, including on stock."""
    seen = []
    tg.send_gcode("G91 G1 X2 F60", check=True, allow="2 mm relative move, returned below")
    try:
        tg.send_raw(b"G21 (partial comment")
        tg.send_raw(b"!")
        tg.send_line(")", check=True)
        end = time.monotonic() + 2
        while time.monotonic() < end:
            seen.append(tg.get("hold"))
            if seen[-1] != 0:
                break
        assert seen and any(state != 0 for state in seen), seen
    finally:
        tg.send_raw(b"\n~")
        tg.wait_idle()
        tg.send_gcode("G91 G1 X-2 F300", check=True, allow="return to start")
        tg.wait_idle()
        tg.send_gcode("G90", check=True)
