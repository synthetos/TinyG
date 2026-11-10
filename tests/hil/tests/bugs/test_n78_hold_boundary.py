"""N78: queue real CAM chords and request a hold exactly at a new block.

The hooks build chooses the timing of the real hold APIs; it never writes speed,
forward differences, geometry, or DDA state. ICE records the inherited state.
"""
from pathlib import Path

import pytest
from tinyg_hil.sender import stream

pytestmark = [pytest.mark.motion, pytest.mark.ice, pytest.mark.persists,
              pytest.mark.resets_board, pytest.mark.dialect('d449'),
              pytest.mark.bug('N78', on=['d449'])]


def test_hold_at_new_cam_block_does_not_panic(tg, ice, keep, record):
    try:
        ice.symbols.resolve('test_hold.line')
    except KeyError:
        pytest.skip('requires the N78 boundary-hold test hook')
    profile = {f'{m}{suffix}': value for m in range(1, 5)
               for suffix, value in (('ma', m-1), ('sa', 1.8), ('mi', 8),
                                     ('tr', 1.25 if m<=3 else 360))}
    profile.update({a+s: v for a in 'xy' for s,v in
                    (('am', 1), ('vm', 20000), ('fr', 20000), ('jm', 5000), ('jd', .05))})
    profile.update(ja=100000, rxm=1, ex=1, spph=0, sl=0, g54x=0, g54y=0)
    keep(*profile)
    resumed = []

    def resume_once(timestamp, status):
        if status.get('stat') == tg.dialect.stat('HOLD') and not resumed:
            tg.send_raw(b'~')
            resumed.append(True)

    try:
        for token, value in profile.items():
            tg.set(token, value, allow='N78 bounded CAM hold profile')
        applied = {token: tg.get(token) for token in profile}
        record('hold_boundary_profile', applied)
        for token,value in profile.items():
            assert applied[token] == pytest.approx(value), (token, applied[token], value)
        tg.send_gcode('G21 G90 G17 G94 G92.1 G54', check=True)
        tg.send_gcode('G28.3 X0 Y0 Z0 A0', check=True, allow='N78 seed unwired bench origin')
        tg.wait_idle()
        names=[f'en.en[{m}].{f}' for m in range(2) for f in ('encoder_steps','steps_run')]
        before=ice.peek_many(names)
        with ice.halted():
            ice.poke('test_hold.fired', 0)
            ice.poke('test_hold.line', 781)
        path=Path(__file__).resolve().parents[2]/'corpus/n78/hold-boundary.nc'
        lines=['G4 P2', *path.read_text().splitlines()]
        for line in lines:
            tg.guard.check_gcode(line, allow='N78 reduced CAM chain in a 3 mm box')
        result=stream(tg, lines, 'g2', timeout=20, on_status=resume_once)
        state=ice.peek_many(['test_hold.fired','test_hold.planned_entry','test_hold.cruise_cap',
                             'test_hold.previous_segment','test_hold.previous_diff'])
        record('hold_boundary', {'state':state,'stream':str(result),'resumed':bool(resumed)})
        assert state['test_hold.fired']==1, state
        # A passing run is useful only if the old block's speed plus its next
        # acceleration step would exceed this new block's limit.
        inherited = state['test_hold.previous_segment'] + state['test_hold.previous_diff']
        assert inherited > state['test_hold.cruise_cap'] + 1, state
        assert 0 < state['test_hold.planned_entry'] <= state['test_hold.cruise_cap'] + .01, state
        assert result.ok, result
        assert resumed, 'hold must reach HOLD and resume'
        after=ice.peek_many(names+['mr.position[0]','mr.position[1]'])
        record('hold_boundary_steps', {'before':before,'after':after})
        for m,end in enumerate((2.6448,1.9142)):
            pulses=sum(after[f'en.en[{m}].{f}']-before[f'en.en[{m}].{f}']
                       for f in ('encoder_steps','steps_run'))
            assert after[f'mr.position[{m}]']==pytest.approx(end,abs=.001)
            assert abs(pulses-end*1280)<=1.5, (m,pulses,end*1280)
    finally:
        ice.poke('test_hold.line', 0)
        tg.reset()
