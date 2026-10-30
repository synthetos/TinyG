"""E139: changing $ex never resets the XON/XOFF state, so after the host has sent XOFF and then
switches flow control off, the board's TX stays paused until a reset.

set_ex (config_app.c:882) only stores xio.enable_flow_control and flag_xoff; it never resets
fc_state_tx. The TX ISR returns while fc_state_tx == FC_IN_XOFF (xio_usb.c:95), and with flag_xoff
cleared the XON trap (xio_usb.c:162) no longer runs. Master has identical logic (config_app.c:894,
xio_usb.c:96/162).
"""
import time

import pytest

from tinyg_hil import NoResponse
from tinyg_hil.protocol import XOFF, XON


@pytest.mark.persists
@pytest.mark.resets_board
@pytest.mark.bug("E139", on=["d440", "d449"])
def test_disabling_flow_control_releases_xoff(tg, keep):
    keep("ex")
    if tg.get("ex") != 1:
        tg.set("ex", 1, allow="XON/XOFF mode for this test; restored by keep")
    tg.send_raw(bytes([XOFF]))
    try:
        resp = tg.send_json({"ex": 0}, timeout=2.0, allow="flow control off; restored by keep")
        assert resp.status == 0
    finally:
        tg.reset()          # the only way out of a stuck XOFF on 449.01; keep() then restores ex


def test_xoff_then_xon_resumes_output(tg):
    if tg.get("ex") != 1:
        pytest.skip("needs XON/XOFF flow control ($ex=1)")
    tg.send_raw(bytes([XOFF]))
    try:
        tg.transport.write(b'{"fb":null}\n')
        with pytest.raises(NoResponse):         # paused: the reply waits in the board's TX buffer
            tg.wait_for(lambda e: e.kind == "response", 0.5)
    finally:
        tg.send_raw(bytes([XON]))
    time.sleep(0.05)
    assert any(e.kind == "response" for e in tg.drain(quiet=0.3))
