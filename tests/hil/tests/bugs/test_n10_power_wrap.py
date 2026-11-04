"""N10: tick rollover must not disable a motor while its segment is stepping.

TinyG wiki TinyG-Configuration-for-Firmware-Version-0.97, $MT: time after last
movement, not an absolute deadline. ICE advances the clock instead of waiting
48.5 days; reset afterwards restores the ordinary clock for other tests.
"""
import time
import pytest
pytestmark=[pytest.mark.motion,pytest.mark.ice,pytest.mark.persists,
            pytest.mark.resets_board,pytest.mark.dialect("d449"),
            pytest.mark.bug("N10",on=[])]
def test_wrap_keeps_running_motor_energized(tg,keep,ice,record):
    if tg.get("1ma")!=0:
        pytest.skip("motor 1 must drive X for this power-state check")
    keep("1pm","mt")
    tg.set("1pm",2,allow="motor power wrap check")
    tg.set("mt",10,allow="motor power wrap check")
    tg.send_gcode("G21 G91",check=True)
    try:
        ice.poke("rtc.rtc_ticks",429496320)  # RTC derives sys_ticks as rtc_ticks * 10
        ice.poke("rtc.sys_ticks",0xfffff000)
        tg.send_gcode("G1 X1 F30",check=True,allow="1 mm power wrap check")
        time.sleep(.25)
        state=ice.peek_many(["st_run.dda_ticks_downcount","st_run.mot[0].power_state","rtc.sys_ticks"])
        record("during move",state)
        assert state["st_run.dda_ticks_downcount"]>0,"did not sample a moving segment"
        assert state["st_run.mot[0].power_state"]!=1,"motor disabled while its segment is stepping"
        tg.wait_idle(timeout=8)
    finally:
        tg.reset()
