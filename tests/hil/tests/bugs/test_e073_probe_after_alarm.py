"""E073: the probing (and homing) callbacks keep queuing moves after an ALARM.

cm_probe_callback (cycle_probing.c:133-142) and cm_homing_callback (cycle_homing.c:189-198) never
check machine_state, and cm_alarm (canonical_machine.c:762-781) leaves cycle_state alone. After the
alarm's hold and flush, a pending G38.2 runs _probing_start and moves the machine while it is in
ALARM; the cycle then exits through cm_cycle_end() -> _exec_program_finalize(), which sets
MACHINE_PROGRAM_STOP (canonical_machine.c:1749) and silently clears the alarm.

Bench: the probe is Z-min with nothing connected; edge forces probe inputs to NO
(switch.c:127-128), so the open input reads untripped and a probe that runs moves its full 1 mm.
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


@pytest.mark.bug("E073", on=[])
def test_pending_probe_does_not_run_after_alarm(tg, keep):
    use_zmin_probe(tg, keep)
    tg.send_gcode("G92 X0 Y0 Z0", check=True)     # work zero here; M2 in the next test clears it
    alarm = tg.dialect.stat("ALARM")
    z0 = get(tg, "mpoz")
    try:
        tg.send_gcode("G1 X5 F100", check=True, allow="5 mm move (3 s), stopped by the alarm")
        tg.send_gcode("G38.2 Z-1 F60", check=True, allow="pending 1 mm probe")
        time.sleep(0.5)
        tg.send_json({"alarm": None})
        # Buggy firmware runs the 1 mm probe (~1 s) after the hold, then drops out of ALARM.
        end = time.monotonic() + 6
        while get(tg, "stat") == alarm and time.monotonic() < end:
            time.sleep(0.25)
        assert get(tg, "stat") == alarm
        assert get(tg, "mpoz") == pytest.approx(z0, abs=0.01)
    finally:
        # the unexecuted probe stays queued under the alarm: flush it before waiting for idle
        tg.transport.write(b"%\n")
        time.sleep(0.5)
        tg.drain()
        tg.clear_alarm()
        idle(tg)
        tg.send_gcode("G90 G1 X0 Y0 Z0 F600", check=True, allow="return to the G92 zero")
        idle(tg)


@pytest.mark.ice
@pytest.mark.profile("switches-no", "homing-x")
@pytest.mark.bug("E073", on=[])
def test_homing_stops_for_good_on_alarm(tg, switches, record):
    """cm_homing_callback queues the next homing move after an alarm's hold; the cycle must end instead."""
    switches.mount("x", "min", at=-50.0)        # far away: the alarm arrives during the search
    try:
        tg.send_gcode("G28.2 X0", check=True, allow="homing search toward a switch 50 mm away")
        time.sleep(1.0)
        tg.send_json({"alarm": None})
        time.sleep(1.5)                            # the hold, the flush, and any wrongly queued move
        p1 = switches.physical_position("x")
        time.sleep(1.5)
        p2 = switches.physical_position("x")
        record("positions", {"after_alarm": p1, "later": p2})
        assert tg.stat() == tg.dialect.stat("ALARM"), "the alarm was cleared by the homing cycle's exit"
        assert tg.get("cycs") == 0 and tg.get("homx") == 0
        assert abs(p2 - p1) < 0.01, "X kept moving after the alarm"
    finally:
        tg.clear_alarm()
        switches.unmount_all()
        tg.send_gcode("G90", check=True)
