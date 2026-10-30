"""Diagnostic: reproduce the XOFF/XON transmit wedge and record what the Atmel-ICE sees (E142, E128).

On firmware without the E142 fix the board stops transmitting after XON when its TX buffer was full;
this records the CPU location and the USB USART/flow-control state at that moment.
"""
import collections
import time

import pytest

from tinyg_hil import NoResponse
from tinyg_hil.protocol import XOFF, XON

pytestmark = [pytest.mark.diagnostic, pytest.mark.ice, pytest.mark.resets_board]
FIELDS = ["us[0].fc_state_tx", "us[0].fc_state_rx", "us[0].fc_char_rx", "us[0].tx_buf_head", "us[0].tx_buf_tail",
          "cs.hard_reset_requested", "xio.enable_flow_control"]
USARTC0 = 0x08A0


def test_xoff_wedge_snapshot(tg, ice, record):
    if tg.get("ex") != 1:
        pytest.skip("needs XON/XOFF flow control ($ex=1)")
    record("before", ice.peek_many(FIELDS))
    try:
        tg.drain(quiet=0.05)
        tg.send_raw(bytes([XOFF]))
        tg.transport.write(b"$$\n")
        time.sleep(0.5)
        tg.send_raw(bytes([XON]))
        lines_after_xon = len(tg.drain(quiet=0.5, limit=3.0))
        with ice.halted():
            regs = {"USARTC0.STATUS": ice.read(USARTC0 + 1, 1)[0], "USARTC0.CTRLA": ice.read(USARTC0 + 3, 1)[0]}
        snapshot = {"lines_after_xon": lines_after_xon, "state": ice.peek_many(FIELDS),
                    "regs": {k: f"0x{v:02x}" for k, v in regs.items()},
                    "pc_samples": collections.Counter(ice.where() for _ in range(20)).most_common(4)}
        record("after_xon", snapshot)
        assert lines_after_xon > 0, f"transmit wedged after XON: {snapshot}"
    finally:
        tg.send_raw(bytes([XON]))
        try:
            tg.reset()
        except NoResponse:
            tg.close()
            time.sleep(1.0)
            tg.transport.open()
            tg.reset()
