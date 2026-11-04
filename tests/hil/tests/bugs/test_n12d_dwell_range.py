"""N12d: G4 P4295 must report its range error, not wrap to an immediate dwell.

NIST RS274NGC v3 3.5.4: P is seconds. TinyG's uint32 microsecond path accepts
at most 4294 seconds; the response must state that implementation limit.
"""
import pytest
pytestmark=[pytest.mark.resets_board,pytest.mark.dialect("d449"),pytest.mark.bug("N12d",on=[])]
def test_long_dwell_is_refused(tg):
    try:
        r=tg.send_line("G4 P4295")
        assert r.status==tg.dialect.status("INPUT_VALUE_RANGE_ERROR"),"long dwell wrapped instead of being refused"
        assert "4294" in str(r.body),"range response must give the maximum seconds"
        assert tg.get("cycs")==0
    finally:
        tg.reset()
