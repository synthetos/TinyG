"""E156: a homing search that never finds its switch ends without telling the host.

cycle_homing.c: when the search move ends with the switch still open, _homing_axis_latch() calls
_homing_abort(), which returns STAT_HOMING_CYCLE_FAILED to the controller's dispatcher; nothing
prints it (the other homing errors go through _homing_error_exit(), which reports). The host saw
the cycle stop with homx 0 and no error.

Bench: nothing closes X-min; travel and backoffs are shrunk so the search moves ~2 mm.
"""
import time

import pytest

from tinyg_hil.driver import NoResponse

pytestmark = [pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449")]

SMALL_HOMING = {"st": 0, "xsn": 1, "xsx": 0, "xtn": 0, "xtm": 2, "xlb": 1, "xzb": 0.5, "xsv": 300, "xlv": 100}


def _body(ev):
    return ev.body if isinstance(ev.body, dict) else {}


def _get(tg, token, timeout=2.0):
    tg.transport.write(('{"%s":null}\n' % token).encode())
    return tg.wait_for(lambda e: e.kind == "response" and token in _body(e), timeout).body[token]


@pytest.mark.bug("E156", on=["d449"])
def test_homing_search_miss_is_reported(tg, keep):
    keep(*SMALL_HOMING)
    for token, value in SMALL_HOMING.items():
        tg.set(token, value)
    mark = len(tg.transport.history)
    tg.send_gcode("G28.2 X0", check=True, allow="homing X over ~2 mm with no switch")
    end = time.monotonic() + 30
    while _get(tg, "cycs") != 0:
        if time.monotonic() > end:
            raise NoResponse("homing cycle did not end")
        time.sleep(0.2)
    tg.drain()
    failed = tg.dialect.status("HOMING_CYCLE_FAILED")
    seen = tg.transport.history[mark:]
    assert any((e.kind == "exception" and e.data["er"].get("st") == failed) or
               (e.kind == "response" and e.data.get("f", [0, 0])[1] == failed) for e in seen), \
        "the failed homing search was not reported"
    assert _get(tg, "homx") == 0
