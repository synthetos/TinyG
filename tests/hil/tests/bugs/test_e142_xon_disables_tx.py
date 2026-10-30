"""E142: an XON from the host disables the USB transmit interrupt, deadlocking a board whose TX buffer is full.

xio/xio_usb.c (RX ISR, XON trap): `USBu.fc_state_tx = FC_IN_XON; USBu.usart->CTRLA = CTRLA_RXON_TXOFF;
// force a TX interrupt` - RXON_TXOFF disables the DRE interrupt instead of enabling it. If the main loop
is waiting in xio_putc_usb() for TX space (buffer filled while the host held XOFF), nothing re-enables the
interrupt: the board never transmits again, and ^X (serviced from the main loop) cannot reset it. Only a
CTS pin change (e.g. reopening the port) re-enables TX. Found with the Atmel-ICE: PC stuck in
xio_putc_usb, USARTC0.CTRLA = 0x20 (DRE interrupt off), fc_state_tx = FC_IN_XON, TX buffer full.
"""
import time

import pytest

from tinyg_hil import NoResponse
from tinyg_hil.protocol import XOFF, XON


@pytest.fixture
def recover(tg):
    yield
    tg.send_raw(bytes([XON]))
    for _ in range(3):
        try:
            tg.reset()
            return
        except NoResponse:
            tg.close()                       # reopening toggles RTS -> CTS pin-change ISR re-enables TX
            time.sleep(1.0)
            tg.transport.open()


@pytest.mark.resets_board
@pytest.mark.bug("E142", on=["d440", "d449"])
def test_output_resumes_after_xon_with_full_tx_buffer(tg, recover):
    if tg.get("ex") != 1:
        pytest.skip("needs XON/XOFF flow control ($ex=1)")
    tg.drain(quiet=0.05)
    tg.send_raw(bytes([XOFF]))
    tg.transport.write(b"$$\n")              # several KB: fills the 250-byte TX buffer while paused
    time.sleep(0.5)
    t_xon = time.monotonic()
    tg.send_raw(bytes([XON]))
    lines = tg.drain(quiet=0.5, limit=10.0)
    assert any(e.t > t_xon for e in lines), "no output at all after XON (TX interrupt left disabled)"
    tg.get("fb")                             # back to JSON mode; the board must answer
    assert tg.get("fb") == pytest.approx(tg.identity["fb"])


def test_output_resumes_after_short_xoff(tg):
    """Control: XOFF/XON around a short reply (TX buffer never fills) works on stock firmware."""
    if tg.get("ex") != 1:
        pytest.skip("needs XON/XOFF flow control ($ex=1)")
    tg.send_raw(bytes([XOFF]))
    tg.transport.write(b'{"fb":null}\n')
    time.sleep(0.2)
    tg.send_raw(bytes([XON]))
    ev = tg.wait_for(lambda e: e.kind == "response", 2.0)
    assert ev.body.get("fb") == pytest.approx(tg.identity["fb"])
