"""N107: earlier commands must not make G28/G30 refuse an admitted line."""
import pytest

pytestmark = [pytest.mark.motion, pytest.mark.resets_board,
              pytest.mark.dialect('d449'), pytest.mark.bug('N107', on=['d449'])]


@pytest.mark.parametrize('stored', ['G28', 'G30'])
def test_dense_stored_move_behind_full_queue(tg, record, stored):
    try:
        tg.send_gcode('G21 G91', check=True)
        tg.send_gcode(stored+'.1', check=True, allow='save current unloaded machine position')
        tg.wait_idle()
        start=float(tg.get('mpox'))
        for _ in range(30):
            tg.send_gcode('G4 P2', check=True, timeout=10)
        line=f'S100 T1 M6 M3 M7 M8 G49 G54 {stored} X1 M30'
        response=tg.send_gcode(line, timeout=40,
                               allow='unloaded 1 mm stored-position test with spindle/coolant outputs')
        record('stored_position_admission', {'line':line, 'status':response.status, 'reply':response.raw})
        assert response.status==0, response.raw
        tg.wait_idle(timeout=75)
        assert float(tg.get('mpox'))==pytest.approx(start,abs=.001)
    finally:
        tg.reset()
