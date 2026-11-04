"""N31: unrepresentable PWM settings are refused without persisting them.

The generic config-init recovery loads invalid saved values through the same
setter, falls back to the profile default, and reports once after TX starts.
The serial test checks SET; the host guard test covers the timer register path.
"""
import pytest
pytestmark=[pytest.mark.dialect('d449'),pytest.mark.persists,
            pytest.mark.bug('N31',on=[])]

@pytest.mark.parametrize('frequency',[0,7,7.5,61,122,244,488,125001])
def test_unrepresentable_frequency_is_not_stored(tg,keep,frequency):
    keep('p1frq')
    before=tg.get('p1frq')
    reply=tg.send_json({'p1frq':frequency},allow='N31 invalid frequency must be refused')
    assert reply.status in (108,109,110),reply.raw
    assert tg.get('p1frq')==pytest.approx(before)
