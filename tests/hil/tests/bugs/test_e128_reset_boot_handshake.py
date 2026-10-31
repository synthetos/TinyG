"""E128: reset/boot handshake. ^X only sets a flag that the main loop services, so a loop stalled on
a full TX buffer cannot be reset; the 449 boot banner is a bare sr object with no r{} or footer, and
it follows the persisted $js, so it can come out as relaxed (non-JSON) syntax.

xio/xio_usb.c:142-145 `if (c == CHAR_RESET) { hw_request_hard_reset(); return; }` only sets
cs.hard_reset_requested (hardware.c:153); the watchdog reset runs from hw_hard_reset_handler
(hardware.c:163-170, controller.c:148), and xio_usb.c:58-59 spins while TX is full. report.c:65-75
prints the banner with json_print_object(NV_HEAD) and no footer (master report.c:88-99 used
json_print_response); the SYSTEM READY line goes through json_serialize/json_relax. Master's reset
path is the same flag-and-loop design.
"""
import json
import time

import pytest

from tinyg_hil import NoResponse
from tinyg_hil.protocol import XOFF, XON


def _is_ready(ev) -> bool:
    # 449 also prints "Loading configs from EEPROM" as a banner-shaped sr line just before.
    return ev.kind == "banner" and isinstance(ev.body, dict) and ev.body.get("msg") == "SYSTEM READY"


@pytest.fixture
def recover(tg):
    """The stalled board can stay silent even after XON; get it back for the next test."""
    yield
    tg.send_raw(bytes([XON]))
    for attempt in range(3):
        try:
            tg.reset()
            return
        except NoResponse:
            # Seen on 449.01: neither XON nor ^X gets through, but closing and reopening the port does.
            tg.close()
            time.sleep(1.0)
            tg.transport.open()


def _reset(tg):
    """^X and return the SYSTEM READY line itself."""
    tg.drain(quiet=0.05)
    tg.send_raw(b"\x18")
    ready = tg.wait_for(_is_ready, 15)
    tg.drain(quiet=0.3)
    return ready


@pytest.mark.resets_board
@pytest.mark.persists
@pytest.mark.bug("E128", on=["d440"])
def test_ctrl_x_resets_while_tx_is_blocked(tg, recover, keep):
    keep("ex")
    tg.set("ex", 1)
    tg.drain(quiet=0.05)
    tg.send_raw(bytes([XOFF]))
    try:
        tg.transport.write(b"$$\n")
        time.sleep(0.5)  # let the listing fill TX, then never release it before the reset verdict
        tg.send_raw(b"\x18")
        ready = tg.wait_for(_is_ready, 20)
        assert _is_ready(ready)
    finally:
        tg.send_raw(bytes([XON]))
    tg.drain(quiet=0.3)
    tg.get("fb")


@pytest.mark.dialect("d449")
@pytest.mark.resets_board
def test_banner_retains_edge_sr_envelope(tg):
    # Edge deliberately uses an unsolicited sr banner, not a command response.
    banner = _reset(tg)
    assert "sr" in banner.data, banner.raw
    assert banner.body["msg"] == "SYSTEM READY"


@pytest.mark.persists
@pytest.mark.resets_board
@pytest.mark.bug("E128", on=["d440"])
def test_banner_is_strict_json_with_relaxed_js(tg, keep):
    keep("js")
    tg.set("js", 0)
    banner = _reset(tg)
    assert tg.get("js") == 0  # strict startup output must not change the selected syntax
    tg.set("js", 1)
    json.loads(banner.raw)
