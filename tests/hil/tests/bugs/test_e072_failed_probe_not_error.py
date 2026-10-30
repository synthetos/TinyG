"""E072: a G38.2 that makes no contact is not reported as an error.

_probing_finish (cycle_probing.c:200) sets PROBE_FAILED, prints {prb} with e:0 (:208-209) and
_probing_finalize_exit returns STAT_OK (:234); the G38.2 line already returned OK at parse time, so a
job continues and the next G10 L20/G92 zeroes at the wrong place. g2 raises
cm_alarm(STAT_PROBE_CYCLE_FAILED) for G38.2. The limit-switch half of the finding
(switch.c:219-222 feedholds on any switch during CYCLE_PROBE) needs a switch stimulus and is not tested.

Bench: the probe is Z-min with nothing connected; edge forces probe inputs to NO
(switch.c:127-128), so the open input reads untripped and the probe always misses.
"""
import time

import pytest

from tinyg_hil.driver import NoResponse

pytestmark = [pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449")]


def _body(ev):
    return ev.body if isinstance(ev.body, dict) else {}


def get(tg, token, timeout=2.0):
    """GET that skips unrelated {"r":...} lines such as the asynchronous {"prb":...} report."""
    tg.transport.write(('{"%s":null}\n' % token).encode())
    return tg.wait_for(lambda e: e.kind == "response" and token in _body(e), timeout).body[token]


def idle(tg, timeout=15.0):
    end = time.monotonic() + timeout
    while get(tg, "cycs") != 0 or get(tg, "qr") != tg.planner_buffers:
        if time.monotonic() > end:
            raise NoResponse(f"not idle after {timeout}s")
        time.sleep(0.1)


def use_zmin_probe(tg, keep):
    """Make Z-min the only probe input (find_probe_switch() errors on two)."""
    for token in ("xsn", "xsx", "ysn", "ysx", "zsx", "asn", "asx"):
        if tg.get(token) == 4:
            keep(token)
            tg.set(token, 0)
    keep("zsn")
    tg.set("zsn", 4)


@pytest.mark.bug("E072", on=["d440", "d449"])
def test_missed_probe_raises_alarm(tg, keep):
    use_zmin_probe(tg, keep)
    tg.send_gcode("G92 X0 Y0 Z0", check=True)     # work zero here; M2 in the next test clears it
    seen = []

    def prb_report(ev):
        seen.append(ev)
        return ev.kind == "response" and "prb" in _body(ev)
    try:
        tg.send_gcode("G38.2 Z-1 F60", check=True, allow="1 mm probe in free air")
        assert tg.wait_for(prb_report, 10).body["prb"]["e"] == 0
        seen += tg.drain(quiet=0.5)
        failed = tg.dialect.status("PROBE_CYCLE_FAILED")
        reported = any(e.kind == "exception" and e.data["er"].get("st") == failed for e in seen)
        assert reported or get(tg, "stat") == tg.dialect.stat("ALARM")
    finally:
        idle(tg)
        tg.drain()
        tg.clear_alarm()
        tg.send_gcode("G90 G1 Z0 F600", check=True, allow="return to the G92 zero")
        idle(tg)
