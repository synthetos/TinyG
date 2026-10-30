"""E130: in G93 the first G1 silently switches the model to G94, and the inverse-time F (minutes) is then used as mm/min.

edge plan_line.c:336-338 `inv_time = gms->feed_rate; gms->feed_rate_mode = UNITS_PER_MINUTE_MODE;` where gms
is &cm.gm (canonical_machine.c:1252 `mp_aline(&cm.gm)`), so {frmo} reads G94 after the move.
Master has the same code (master plan_line.c:331-333). The follow-on symptom (next F-less G1 crawling at
1/F mm/min) is not exercised because it would take an hour on stock firmware.
"""
import pytest

MOVE = "inverse-time test (1 mm X in 1 s), returns to start"


@pytest.mark.motion
@pytest.mark.bug("E130", on=["d440"])
def test_g93_survives_a_feed_move(tg):
    x0 = tg.get("mpox")
    try:
        tg.send_gcode("G93", check=True)
        tg.send_gcode("G91 G1 X1 F60", check=True, allow=MOVE)      # F60 = 1/60 min
        tg.wait_idle()
        assert tg.get("frmo") == 0
        tg.send_gcode("G90", check=True)  # no axes: no inverse-time F required
        tg.send_gcode("M2", check=True)
    finally:
        tg.send_gcode("G94", check=True)
        tg.send_gcode(f"G90 G53 G0 X{x0:.4f}", allow=MOVE)
        tg.wait_idle()


@pytest.mark.motion
@pytest.mark.bug('E130', on=['d440'])
@pytest.mark.parametrize('first',['G91 G1 X0.1 F60','F60'])
def test_inverse_time_feed_does_not_leak_to_another_block(tg,first):
    import time
    x0=tg.get('mpox')
    try:
        tg.send_gcode('G21 G93',check=True)
        tg.send_gcode(first,check=True,allow=MOVE)
        if first.startswith('G91'):tg.wait_idle()
        response=tg.send_gcode('G91 G1 X0.1',allow=MOVE)
        assert response.status!=0, 'G93 motion without F was accepted'
    finally:
        # Unfixed firmware can accept a very slow move; stop it before return.
        tg.send_raw(b'!');time.sleep(.3);tg.send_raw(b'%');time.sleep(.2)
        tg.send_gcode('G94 G90',check=True)
        tg.send_gcode(f'G53 G0 X{x0:.4f}',allow=MOVE)
        tg.wait_idle()
