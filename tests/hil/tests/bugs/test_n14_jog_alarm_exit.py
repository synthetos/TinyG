"""N14: an alarm during a jog must remain latched after jog cleanup.

Unlike normal cycle completion, alarm cleanup must not set PROGRAM_STOP.
Covers the alarm action, real-time ^D job kill, and a hooks-driven limit edge.
"""
import time
import pytest
pytestmark=[pytest.mark.motion,pytest.mark.ice,pytest.mark.persists,pytest.mark.resets_board,
 pytest.mark.dialect("d449"),pytest.mark.bug("N14",on=[])]
@pytest.mark.parametrize("trigger", ["alarm", "ctrl-d", "limit"])
def test_alarm_survives_jog_cleanup(tg,keep,ice,record,switches,trigger):
    if trigger=="limit" and "test_hook" not in ice.symbols.sym:
        pytest.skip("limit case requires TEST_HOOKS=1")
    keep("xvm","rxm","xsn","st")
    if trigger=="limit":
        tg.set("st",0,allow="N14 normally open limit input")
        switches.normally_closed=False
        switches.open("x","min")
        tg.set("xsn",2,allow="N14 X-min limit during jog")
    tg.set("xvm",120,allow="bounded jog speed")
    tg.set("rxm",1,allow="control commands during jog")
    tg.send_gcode("G21 G90 G28.3 X0",check=True,allow="seed X for bounded jog")
    try:
        tg.send_json({"jogx":4},allow="4 mm bounded jog",check=True)
        time.sleep(.2)
        assert ice.peek("cm.cycle_state") == 4, "jog did not start"
        if trigger=="alarm":tg.send_json({"alarm":None})
        elif trigger=="ctrl-d":tg.send_raw(b"\x04")
        else:switches.close("x","min")
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            state=ice.peek_many(["cm.machine_state","cm.cycle_state","cm.hold_state"])
            if state["cm.cycle_state"]==0:break
            time.sleep(.1)
        record("after jog alarm",state)
        assert state["cm.cycle_state"]==0,"jog cleanup did not finish"
        assert state["cm.machine_state"]==tg.dialect.stat("ALARM"),"jog silently cleared its alarm"
        tg.send_json({"clear":None},check=True)
    finally:
        if trigger=="limit":switches.open("x","min")
        tg.reset()
