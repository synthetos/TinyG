"""N14: an alarm during a jog must remain latched after jog cleanup.

Unlike normal cycle completion, alarm cleanup must not set PROGRAM_STOP.
No physical switch is needed: the ordinary {alarm:null} action raises it.
"""
import time
import pytest
pytestmark=[pytest.mark.motion,pytest.mark.ice,pytest.mark.persists,pytest.mark.resets_board,
 pytest.mark.dialect("d449"),pytest.mark.bug("N14",on=["d449"])]
def test_alarm_survives_jog_cleanup(tg,keep,ice,record):
    keep("xvm","rxm")
    tg.set("xvm",120,allow="bounded jog speed")
    tg.set("rxm",1,allow="control commands during jog")
    tg.send_gcode("G21 G90 G28.3 X0",check=True,allow="seed X for bounded jog")
    try:
        tg.send_json({"jogx":4},allow="4 mm bounded jog",check=True)
        time.sleep(.2)
        assert ice.peek("cm.cycle_state") == 4, "jog did not start"
        tg.send_json({"alarm":None})
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
        tg.reset()
