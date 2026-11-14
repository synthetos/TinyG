"""Live spindle requests must change only PWM speed, never start an off spindle."""
import time
import pytest

pytestmark = [pytest.mark.dialect('d449'), pytest.mark.resets_board]


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


@pytest.mark.parametrize('word,expected', [('0.05',.05),('0.050',.05),('5e-2',.05),('0.051',.051),('2.0',2)])
def test_factor_endpoints(tg, word, expected):
    try:
        tg.reset()
        assert tg.send_line('{"sso":'+word+'}').status == 0
        assert tg.get('sso') == pytest.approx(expected)
        assert tg.get('spe') == 0
        lines = tg.send_text('$sso')
        assert any('[sso]' in line for line in lines)
    finally:
        tg.reset()


@pytest.mark.parametrize('word', ['0.049', '0.04999999', '2.001'])
def test_factor_rejected_without_change(tg, word):
    try:
        tg.reset()
        assert tg.send_line('{"sso":'+word+'}').status == 110
        assert tg.get('sso') == 1
    finally:
        tg.reset()


@pytest.mark.parametrize('p', ['-1','0.5','2'])
def test_m51_invalid_parameter(tg, p):
    try:
        tg.reset()
        assert tg.send_line('M51 P'+p).status == 161
        assert tg.get('sse') == 1
    finally:
        tg.reset()


def test_ordered_permissions_and_operator_gates(tg):
    try:
        tg.reset()
        tg.send_gcode('G4 P1', check=True)
        tg.send_gcode('M49', check=True)
        assert tg.get('sse') == 1 and tg.get('mfe') == 1
        wait_value(tg, 'sse', 0)
        assert tg.get('mfe') == 0
        tg.send_line('{"sso":1.5,"ssoe":1,"m48e":1}')
        assert tg.get('sse') == 0  # operator never bypasses the program
        tg.send_gcode('M51', check=True)
        wait_value(tg, 'sse', 1)
        assert tg.get('mfe') == 0
        tg.send_gcode('M51 P0', check=True)
        wait_value(tg, 'sse', 0)
        tg.send_gcode('M50 P1', check=True)
        wait_value(tg, 'mfe', 1)
        assert tg.get('sse') == 0
        tg.send_gcode('M48', check=True)
        wait_value(tg, 'sse', 1)
        for master, spindle in ((0,0),(0,1),(1,0),(1,1)):
            assert tg.send_line('{"m48e":%d,"ssoe":%d}' % (master,spindle)).status == 0
            assert tg.get('sse') == (master and spindle)
    finally:
        tg.reset()


@pytest.mark.parametrize('ending', ['M2','M30','kill','reset'])
def test_job_reset(tg, ending):
    try:
        tg.reset()
        tg.set('sso', .5)
        tg.set('ssoe', 0)
        tg.send_gcode('M51 P0', check=True)
        wait_value(tg, 'sse', 0)
        if ending == 'reset':
            tg.reset()
        elif ending == 'kill':
            tg.send_raw(b'\x04')
        else:
            tg.send_gcode('G4 P1', check=True)
            tg.send_gcode(ending, check=True)
            assert tg.get('sso') == .5
        wait_value(tg, 'sso', 1)
        assert tg.get('ssoe') == 1 and tg.get('sse') == 1
        assert tg.get('spe') == 0
    finally:
        tg.reset()


@pytest.mark.persists
@pytest.mark.parametrize('direction,prefix', [('M3','p1c'),('M4','p1w')])
def test_clamp_and_alarm(tg, keep, direction, prefix):
    try:
        tg.reset()
        for suffix,value in [('sl',1000),('sh',10000),('pl',.1),('ph',.9)]:
            keep(prefix+suffix)
            tg.set(prefix+suffix,value,allow='known spindle range')
        tg.send_gcode('S8000 '+direction,check=True,allow='spindle test')
        wait_value(tg,'sps',8000)
        tg.set('sso',2)
        wait_value(tg,'sps',10000)
        assert tg.get('scp') == 1
        tg.set('sso',.5)
        wait_value(tg,'sps',4000)
        assert tg.get('scp') == 0
        tg.set('sso',.05)
        wait_value(tg,'sps',1000)
        assert tg.get('scp') == 1
        tg.send_json({'alarm':None})
        wait_value(tg,'spe',0)
        assert tg.send_line('{"sso":2}').status != 0
        assert tg.get('sps') == 0
    finally:
        tg.reset()


@pytest.mark.motion
@pytest.mark.timeout(60)
def test_factor_during_motion_and_hold(tg):
    try:
        tg.reset()
        tg.send_gcode('S4000 M3',check=True,allow='spindle test')
        tg.send_gcode('G21 G91 G94 G1 X80 F600',check=True,allow='hold test move')
        time.sleep(.5)
        tg.send_raw(b'!')
        wait_value(tg,'hold',6)
        tg.set('sso',.5)
        expected = min(max(2000,tg.get('p1csl')),tg.get('p1csh'))
        wait_value(tg,'sps',expected)
        assert tg.get('hold') == 6 and tg.get('spe') == 1
        tg.send_raw(b'%')
        wait_value(tg,'hold',0)
        assert tg.get('spe') == 1
        tg.send_gcode('M5',check=True,allow='end spindle test')
        wait_value(tg,'spe',0)
    finally:
        tg.reset()
