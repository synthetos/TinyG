"""Ordered program permission and RAM-only feed request lifecycle."""
import time

import pytest
from test_feed_advisor_step3 import rt

pytestmark = [pytest.mark.motion, pytest.mark.resets_board,
              pytest.mark.dialect('d449'), pytest.mark.timeout(45)]


def read(tg, token):
    reply = rt(tg, {token: None})
    assert reply.status == 0, reply
    return reply.body['rt'][token]


def wait_value(tg, token, wanted):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if read(tg, token) == pytest.approx(wanted):
            return
        time.sleep(.05)
    pytest.fail(f'{token} never became {wanted}')


def test_program_permission_cannot_be_bypassed_by_live_request(tg, record):
    try:
        tg.reset()
        assert read(tg, 'mfe') == 1
        assert rt(tg, {'mfo': .5}).status == 0
        tg.send_gcode('G4 P1', check=True)
        tg.send_gcode('M49', check=True)
        # The program permission is queued behind the dwell.
        assert read(tg, 'mfe') == 1
        wait_value(tg, 'mfe', 0)
        assert rt(tg, {'mfo': 1.5, 'mfe': 1}).status == 0
        assert read(tg, 'mfe') == 0
        assert read(tg, 'mfo') == pytest.approx(1.5)
        for command, expected in [('M48', 1), ('M50 P0', 0), ('M50', 1)]:
            tg.send_gcode(command, check=True)
            wait_value(tg, 'mfe', expected)
        assert tg.send_gcode('M50 P2').status != 0
        assert read(tg, 'mfe') == 1
        record('program_permission', {'requested': read(tg, 'mfo'), 'enabled': read(tg, 'mfe')})
    finally:
        tg.reset()


@pytest.mark.parametrize('ending', ['M2', 'M30', 'kill', 'reset'])
def test_job_end_restores_unity_permission_and_disables_bytes(tg, ending, record):
    try:
        tg.reset()
        assert rt(tg, {'mfo': .5, 'mfe': 0}).status == 0
        assert rt(tg, {'ovb': 1}).status == 0
        tg.send_gcode('M49', check=True)
        wait_value(tg, 'mfe', 0)
        if ending == 'reset':
            tg.reset()
        elif ending == 'kill':
            tg.send_raw(b'\x04')
        else:
            tg.send_gcode(ending, check=True)
        wait_value(tg, 'mfo', 1)
        assert read(tg, 'mfc') == 1
        assert read(tg, 'mfe') == 1
        assert read(tg, 'ovb') == 0
        assert not read(tg, 'mcp')
        record('override_end', {'ending': ending, 'requested': read(tg, 'mfo')})
    finally:
        tg.reset()
