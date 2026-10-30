"""E032: T plus M6 records the previously selected tool, and G43 Hn raises ALARM.

edge canonical_machine.c:1292-1297 cm_change_tool() snapshots cm.gm.tool_select at parse time, but
cm_select_tool() (:1280-1290) only sets it when its queued command runs, so `T5 M6` sets the old tool.
gcode_parser.c:325-331 sends plain G43 to STAT_GCODE_COMMAND_UNSUPPORTED, which alarms at :427-428.
Master has the same tool code (master canonical_machine.c:1033-1050) but only errors on G43.
"""
import pytest

M6 = "M6 only records the tool number on this firmware"


@pytest.mark.motion
@pytest.mark.bug("E032", on=["d440"])
def test_t_and_m6_on_one_line(tg):
    tg.send_gcode("T0 M6", check=True, allow=M6)
    tg.wait_idle()
    tg.send_gcode("T5 M6", check=True, allow=M6)
    tg.wait_idle()
    assert tg.get("tool") == 5


@pytest.mark.motion
@pytest.mark.bug("E032", on=["d440"])
def test_t_and_m6_on_separate_lines_behind_dwell(tg):
    # With an empty planner the queued T runs before M6 is parsed; a dwell ahead of it exposes the bug.
    tg.send_gcode("T0 M6", check=True, allow=M6)
    tg.wait_idle()
    tg.send_gcode("G4 P1", check=True)
    tg.send_gcode("T3", check=True)
    tg.send_gcode("M6", check=True, allow=M6)
    tg.wait_idle()
    assert tg.get("tool") == 3


@pytest.mark.bug("E032", on=["d449"])
def test_g43_h_does_not_alarm(tg):
    tg.send_gcode("G43 H1")
    assert tg.stat() != tg.dialect.stat("ALARM")
