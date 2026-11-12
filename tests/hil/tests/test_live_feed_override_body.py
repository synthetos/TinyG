"""Long straight feed changes through the main-loop future segment handoff.

This checks wire response, sustained speed and reported endpoint. Physical pulse
counts and interrupt timing need the separate instrumented bench qualification.
"""
import time

import pytest
from test_feed_advisor_step3 import rt

pytestmark = [pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect('d449'),
              pytest.mark.timeout(90)]


def wait_speed(tg, wanted, rows, timeout=5):
    deadline = time.monotonic() + timeout
    consecutive = 0
    while time.monotonic() < deadline:
        value = float(tg.get('vel'))
        rows.append((time.monotonic(), value))
        consecutive = consecutive + 1 if abs(value-wanted) <= 1 else 0
        if consecutive >= 3:
            return
        time.sleep(.1)
    pytest.fail(f'no sustained {wanted} mm/min: {rows}')


@pytest.mark.parametrize('factor', [.5, 1.5])
def test_long_body_changes_speed_and_keeps_endpoint(tg, keep, record, factor):
    profile = dict(xfr=1200, xvm=1200, xjm=500, sl=0)
    keep(*profile)
    rows = []
    try:
        for token, value in profile.items():
            tg.set(token, value, allow='unloaded straight override test')
        assert rt(tg, {'mfo': 1, 'mfe': 1}).status == 0
        tg.send_gcode('G21 G90 G94 G64 G92.1 G54', check=True)
        tg.send_gcode('G28.3 X0', check=True, allow='declare unloaded override origin')
        tg.wait_idle()
        # Use relative distance so a saved work offset cannot move the endpoint.
        tg.send_gcode('G91 G1 X100 F600', check=True, allow='100 mm unloaded override move')
        wait_speed(tg, 600, rows)
        request_time = time.monotonic()
        reply = rt(tg, {'mfo': factor})
        assert reply.status == 0, reply
        wait_speed(tg, 600*factor, rows)
        tg.wait_idle(timeout=35)
        assert float(tg.get('mpox')) == pytest.approx(100, abs=.001)
        record('body_override', dict(factor=factor, requested_at=request_time, speeds=rows,
                                    endpoint=tg.get('mpox')))
    finally:
        tg.send_raw(b'!%')
        time.sleep(.3)
        assert rt(tg, {'mfo': 1, 'mfe': 0}).status == 0
        tg.send_raw(b'~')
        tg.wait_idle(timeout=10)
        tg.send_gcode('G90', check=True)
