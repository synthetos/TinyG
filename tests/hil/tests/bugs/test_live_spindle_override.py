"""Live spindle requests must change only PWM speed, never start an off spindle."""
import time
import pytest

pytestmark = [pytest.mark.dialect('d449'), pytest.mark.resets_board,
              pytest.mark.xfail(strict=True, reason='live spindle override not implemented')]


def wait_value(tg, token, expected):
    end = time.monotonic() + 3
    while time.monotonic() < end:
        value = tg.get(token)
        if value == pytest.approx(expected):
            return
        time.sleep(.03)
    assert value == pytest.approx(expected), (token, value, expected)


def test_spindle_factor_and_stopped_output(tg):
    try:
        tg.reset()
        assert tg.send_line('{"sso":0.5}').status == 0
        assert tg.get('sso') == .5
        assert tg.get('ssoe') == 1
        assert tg.get('sse') == 1
        assert tg.get('spe') == 0
        assert tg.get('sps') == 0
    finally:
        tg.reset()


@pytest.mark.persists
def test_live_spindle_speed_and_permission(tg, keep):
    try:
        tg.reset()
        for k, v in {'p1csl':0, 'p1csh':10000, 'p1cpl':0, 'p1cph':1}.items():
            keep(k)
            tg.set(k, v, allow='known spindle range')
        tg.send_gcode('S4000 M3', check=True, allow='spindle override test')
        wait_value(tg, 'sps', 4000)
        assert tg.send_line('{"sso":0.5}').status == 0
        wait_value(tg, 'sps', 2000)
        tg.send_gcode('M49', check=True)
        wait_value(tg, 'sps', 4000)
        tg.send_gcode('M51 P1', check=True)
        wait_value(tg, 'sps', 2000)
        tg.send_gcode('M5', check=True)
        wait_value(tg, 'spe', 0)
        tg.set('sso', 2)
        assert tg.get('spe') == 0 and tg.get('sps') == 0
    finally:
        tg.reset()
