"""Feedhold and queue flush affect motion only; explicit output commands still work."""
import time
import pytest


def _wait_still(tg, timeout=5):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if tg.stat() == tg.dialect.stat('HOLD') and tg.get('hold') == tg.dialect.hold('HOLD'):
            return tg.get('posx')
        time.sleep(.03)
    raise AssertionError('feedhold did not reach stopped HOLD')


def _outputs(tg):
    return {k: tg.get(k) for k in ('spe', 'spd', 'spc', 'sps', 'cof', 'com')}


@pytest.mark.motion
@pytest.mark.dialect('d449')
@pytest.mark.parametrize('flush', [False, True], ids=['resume', 'flush'])
def test_hold_preserves_outputs(tg, flush):
    tg.send_gcode('G21 G94 G91 M3 S1000', check=True, allow='unloaded output test')
    tg.send_gcode('M7', check=True, allow='mist output test')
    tg.send_gcode('M8', check=True, allow='flood output test')
    tg.wait_idle()
    before = _outputs(tg)
    assert before['spe'] == 1 and before['cof'] == 1 and before['com'] == 1
    try:
        tg.send_gcode('G1 X10 F120', check=True, allow='10 mm hold test')
        time.sleep(.4)
        tg.send_raw(b'!')
        held = _wait_still(tg)
        assert _outputs(tg) == before
        time.sleep(.3)
        assert _outputs(tg) == before
        tg.send_raw(b'%' if flush else b'~')
        if not flush:
            # There is no spindle restart delay: movement must resume promptly.
            end = time.monotonic() + .8
            while time.monotonic() < end and abs(tg.get('posx')-held) < .002:
                time.sleep(.02)
            assert abs(tg.get('posx')-held) >= .002
        tg.wait_idle(timeout=10)
        assert _outputs(tg) == before
        if flush:
            assert abs(tg.get('posx')-held) <= .002
        tg.send_gcode('M5 M9', check=True, allow='explicit output stop')
        tg.wait_idle()
        stopped = _outputs(tg)
        assert stopped['spe'] == stopped['cof'] == stopped['com'] == 0
    finally:
        tg.send_raw(b'!')
        time.sleep(.3)
        tg.send_raw(b'%')
        tg.wait_idle(timeout=10)
        tg.send_gcode('G90 M5 M9', check=True, allow='restore output state')
