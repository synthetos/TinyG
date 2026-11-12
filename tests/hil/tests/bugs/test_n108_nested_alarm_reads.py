"""N108: read-only groups remain available after job kill enters alarm."""
import time

import pytest

pytestmark = [pytest.mark.resets_board, pytest.mark.dialect('d449'),
              pytest.mark.bug('N108', on=[])]


@pytest.mark.parametrize('group,token,flat', [('x', 'vm', 'xvm'), ('1', 'mi', '1mi')])
def test_nested_read_after_job_kill(tg, record, group, token, flat):
    try:
        tg.reset()
        expected = tg.get(flat)
        tg.send_raw(b'\x04')
        deadline = time.monotonic() + 5
        while tg.stat() != tg.dialect.stat('ALARM'):
            assert time.monotonic() < deadline, 'job kill did not enter alarm'
            time.sleep(.05)
        reply = tg.send_json({group: {token: None}})
        record('alarm_group_read', {'group': group, 'status': reply.status, 'reply': reply.raw})
        assert reply.status == 0, reply.raw
        assert reply.body[group][token] == pytest.approx(expected)
        assert tg.send_json({group: {token: expected}}).status == 204
        assert tg.get(flat) == pytest.approx(expected)
    finally:
        tg.reset()
