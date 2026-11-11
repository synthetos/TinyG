"""A dwell is RUN even when all planner slots are free; program pauses stay paused."""
import time
import pytest

pytestmark = [pytest.mark.dialect('d449'), pytest.mark.resets_board, pytest.mark.timeout(30)]


@pytest.mark.bug('N99', on=[])
@pytest.mark.parametrize('preamble,tail', [('M2', ''), ('M2', 'M2'), ('M5 M9', 'M5 M9')],
                         ids=['lone-after-end', 'before-end', 'output-commands'])
def test_dwell_remains_running(tg, record, preamble, tail):
    tg.send_gcode(preamble, check=True)
    tg.wait_idle()
    before = tg.get('spdp')
    rows = []
    try:
        tg.send_gcode('G4 P2', check=True)
        start = time.monotonic()
        if tail:
            tg.send_gcode(tail, check=True)
        for _ in range(4):
            stat = tg.get('stat')
            qr = tg.get('qr')
            refused = tg.send_json({'spdp': before}).status
            rows.append((time.monotonic()-start, stat, qr, refused))
            assert stat == tg.dialect.stat('RUN'), rows
            assert 0 <= qr <= tg.planner_buffers, rows  # free slots, not unfinished runtime work
            assert refused == 117, rows
            time.sleep(0.15)
        if not tail:
            assert rows[-1][2] == tg.planner_buffers, rows
        # wait_idle must wait for the timer, not return on qr=32.
        tg.wait_idle(timeout=5)
        assert time.monotonic()-start >= 1.7, rows
        assert tg.get('qr') == tg.planner_buffers
        assert tg.send_json({'spdp': before}).status == 0
    finally:
        record('dwell_status', rows)
        tg.reset()  # also bounds cleanup when the stock assertion fails mid-dwell


@pytest.mark.parametrize('stop', ['M0', 'M1'])
def test_program_stop_does_not_become_run(tg, keep, stop):
    keep('os')
    tg.set('os', 1)
    try:
        tg.send_gcode(stop, check=True)
        deadline = time.monotonic()+3
        while tg.get('stat') != tg.dialect.stat('PROGRAM_STOP'):
            assert time.monotonic() < deadline
            time.sleep(0.02)
        tg.send_gcode('G4 P0.2', check=True)
        time.sleep(0.1)
        assert tg.get('stat') == tg.dialect.stat('PROGRAM_STOP')
        tg.send_raw(b'~')
        tg.wait_idle(timeout=5)
    finally:
        tg.reset()
