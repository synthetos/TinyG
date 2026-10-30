"""E129: G80 ends the cycle synchronously while queued moves keep running, so stat drops to 3 mid-move.

edge gcode_parser.c:349 maps G80 to MOTION_MODE_CANCEL and :534 calls cm_cancel_motion_mode(), which calls
cm_cycle_end() (canonical_machine.c:1118-1123). cm_cycle_end runs _exec_program_finalize directly
(:1784-1790, not queued like cm_program_stop at :1792-1796), setting machine_state PROGRAM_STOP and cycle OFF
while the arc is still executing. Introduced by 61adc2f2 'added cycle end to G80'; master G80 only sets the
model (master gcode_parser.c:507). Arcs are used because G2/G3 do start a cycle on 449.01 (plan_arc.c:254),
unlike G0/G1 (E033).
"""
import time

import pytest

MOVE = "half-circle G2 to X+2 (radius 1 mm) at F120, then back"


def _stat_while_moving(tg, duration):
    stats, end = [], time.monotonic() + duration
    while time.monotonic() < end:
        s = tg.send_json({"stat": None, "vel": None}, check=True).body
        if s["vel"] > 0:
            stats.append(s["stat"])
    return stats


def _arc(tg, then_g80):
    tg.send_gcode("G17 G91 G2 X2 Y0 I1 J0 F120", check=True, allow=MOVE)     # ~1.6 s
    try:
        time.sleep(0.3)
        if then_g80:
            tg.send_gcode("G80", check=True)
        stats = _stat_while_moving(tg, 1.0)
        assert stats, "never saw vel > 0"
        assert set(stats) == {tg.dialect.stat("RUN")}, f"stat while moving: {sorted(set(stats))}"
    finally:
        tg.wait_idle()
        tg.send_gcode("G91 G1 X-2 F600", allow=MOVE)
        tg.wait_idle()


@pytest.mark.motion
@pytest.mark.dialect("d449")
def test_g80_during_arc_keeps_run_state(tg):
    _arc(tg, then_g80=True)


@pytest.mark.motion
@pytest.mark.dialect("d449")
def test_arc_reports_run(tg):
    _arc(tg, then_g80=False)
