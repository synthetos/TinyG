"""E141: a (MSG ...) comment alone on a line is dropped on edge.

gc_gcode_parser() returns early when the normalized block is empty (gcode_parser.c:61-63),
before the message is queued at :65-68, so the msg never reaches the response.
"""
import pytest


@pytest.mark.bug("E141", on=["d449"])
def test_msg_alone_on_line(tg):
    assert tg.send_gcode("(MSG hello)", check=True).body.get("msg", "").strip() == "hello"


def test_msg_with_gcode(tg):
    assert tg.send_gcode("G21 (MSG hello)", check=True).body.get("msg", "").strip() == "hello"
