"""N06 controls already fixed on edge-next; run alone to avoid prior-case state.

NIST RS274NGC v3 3.5.4 requires negative P to fail. E158 already requires positive
jerk. These are ordinary regressions, not expected failures on current edge-next.
"""
import pytest
pytestmark=[pytest.mark.dialect("d449"), pytest.mark.persists]

def test_negative_dwell_is_refused(tg):
    reply=tg.send_gcode("G4 P-1")
    assert reply.status==tg.dialect.status("P_WORD_IS_NEGATIVE"), reply.raw
    tg.wait_idle(timeout=2)
    assert tg.get("qr")==32

def test_zero_jerk_is_refused(tg, keep):
    keep("xjm")
    before=tg.get("xjm")
    reply=tg.send_json({"xjm":0})
    assert reply.status==tg.dialect.status("INPUT_LESS_THAN_MIN_VALUE"), reply.raw
    assert tg.get("xjm")==pytest.approx(before)
