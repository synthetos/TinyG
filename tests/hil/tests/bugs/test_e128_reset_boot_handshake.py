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
@pytest.mark.bug("E128", on=["d440", "d449"])
def test_ctrl_x_resets_while_tx_is_blocked(tg, recover):
    if tg.get("ex") != 1:
        pytest.skip("pausing the board's output needs XON/XOFF flow control ($ex=1)")
    tg.drain(quiet=0.05)
    tg.send_raw(bytes([XOFF]))          # host pauses the board's TX
    try:
        tg.transport.write(b"$$\n")     # several KB of text: the main loop blocks once TX is full
        time.sleep(0.5)
        tg.send_raw(b"\x18")
        time.sleep(1.0)
    finally:
        t_xon = time.monotonic()
        tg.send_raw(bytes([XON]))
    events, end = [], time.monotonic() + 20
    while not events or not _is_ready(events[-1]):
        ev = tg.transport.next_event(end - time.monotonic())
        assert ev is not None, "no banner within 20 s of ^X"
        events.append(ev)
    tg.drain(quiet=0.3)
    tg.get("fb")                        # leave text mode if the board is still in it
    # A reset taken at ^X discards the rest of the $$ listing; one taken only after the loop
    # resumes prints all of it first.
    late = [e.raw for e in events if e.kind == "text" and e.raw.startswith("[") and e.t > t_xon + 0.2]
    assert len(late) <= 2, f"{len(late)} lines of $$ output after XON, e.g. {late[:3]}"


@pytest.mark.dialect("d449")
@pytest.mark.resets_board
@pytest.mark.bug("E128", on=["d449"])
def test_banner_is_response_with_footer(tg):
    banner = _reset(tg)
    assert "r" in banner.data and "f" in banner.data, banner.raw


@pytest.mark.persists
@pytest.mark.resets_board
@pytest.mark.bug("E128", on=["d440", "d449"])
def test_banner_is_strict_json_with_relaxed_js(tg, keep):
    keep("js")
    tg.set("js", 0)
    banner = _reset(tg)
    tg.set("js", 1)
    json.loads(banner.raw)
