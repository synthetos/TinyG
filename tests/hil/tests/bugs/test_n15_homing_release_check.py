"""N15: a switch held active after search must fail latch, not establish zero.

TinyG wiki 'Homing and Limits Setup and Troubleshooting' requires latch to clear
the switch before zero backoff. This uses a TEST_HOOKS soft switch held closed;
no electrical timing claim is made. Failure status is HOMING_CYCLE_FAILED (240).
"""
import time
import pytest
pytestmark=[pytest.mark.motion,pytest.mark.ice,pytest.mark.persists,pytest.mark.resets_board,
 pytest.mark.dialect("d449"),pytest.mark.profile("switches-no"),pytest.mark.bug("N15",on=["d449"])]
def test_latch_must_release_switch(tg,keep,switches,ice,record):
    if "test_hook" not in ice.symbols.sym:
        pytest.skip("requires TEST_HOOKS=1 switch injection")
    settings={"xsn":1,"xsx":2,"xtn":-2,"xtm":2,"xsv":120,"xlv":60,"xlb":1,"xzb":.2}
    keep(*settings)
    for k,v in settings.items():tg.set(k,v,allow="small homing envelope")
    try:
        switches.unmount_all();switches.open("x","min");switches.open("x","max")
        time.sleep(.1)
        tg.send_gcode("G21 G90 G28.3 X0",check=True,allow="seed X machine position")
        mark=len(tg.transport.history)
        tg.send_gcode("G28.2 X0",check=True,allow="bounded homing with stuck latch switch")
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            if ice.peek("mr.position[0]") < -.15:break
            time.sleep(.05)
        else:pytest.fail("homing search never started")
        switches.close("x","min")  # held active for the entire latch and backoff
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            state=ice.peek_many(["cm.cycle_state","cm.homed[0]","sw.s[0].state","mr.position[0]"])
            if state["cm.cycle_state"]==0:break
            time.sleep(.1)
        record("stuck latch result",state)
        assert state["cm.cycle_state"]==0,"homing failed to terminate"
        assert state["sw.s[0].state"]==1,"test did not hold the switch active"
        assert state["cm.homed[0]"]==0,"homing accepted a switch that never released"
        tg.drain()
        failed=tg.dialect.status("HOMING_CYCLE_FAILED")
        assert any((e.kind=="exception" and e.data["er"].get("st")==failed) or
                   (e.kind=="response" and e.data.get("f",[0,0])[1]==failed)
                   for e in tg.transport.history[mark:]),"failed latch was not reported"
    finally:
        tg.reset();switches.open("x","min");switches.open("x","max")
