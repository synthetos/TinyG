"""GitHub #186: G38.2 right after a tool change must run and report, not hang.

Report (2016): after `T1 M6`, a following `G38.2` locked the board up. On edge the matching causes were fixed in
450.01: JSON-wrapped G-code no longer overtakes a pending probe or queued commands (E034, planner headroom), a block
that queues several commands can no longer PANIC "Buffer full fatal" (E088), and the probe waits for earlier moves
(E025). M6 itself only makes T the current tool (wiki Gcode-Support), it does not pause.

Bench: Z-min is the only probe input with nothing connected, so each 1 mm G38.2 runs its full travel and reports
e=0 (not tripped) and raises the documented probe-failed alarm (status 250, wiki TinyG-Alarms), which the
test clears, as test_e025 does. Both plain and JSON-wrapped tool changes are tried, each followed
by two probes; the tool change must be accepted and select its tool, every probe must report, and the board
must answer and end idle at the start position. #183 (contact, then G28.3 and queued motion) needs its own test.
"""
import time

import pytest

from tinyg_hil.driver import NoResponse

pytestmark = [pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"), pytest.mark.timeout(120)]


def _body(ev):
    return ev.body if isinstance(ev.body, dict) else {}


def _get(tg, token, timeout=2.0):
    tg.transport.write(('{"%s":null}\n' % token).encode())
    return tg.wait_for(lambda e: e.kind == "response" and token in _body(e), timeout).body[token]


def _wait_prb(tg, timeout=10.0):
    return tg.wait_for(lambda e: e.kind in ("probe", "response") and "prb" in _body(e), timeout).body["prb"]


def _idle(tg, timeout=15.0):
    end = time.monotonic() + timeout
    while _get(tg, "cycs") != 0 or _get(tg, "qr") != tg.planner_buffers:
        if time.monotonic() > end:
            raise NoResponse(f"not idle after {timeout}s")
        time.sleep(0.1)


@pytest.fixture
def zmin_probe(tg, keep):
    for token in ("xsn", "xsx", "ysn", "ysx", "zsx", "asn", "asx"):
        if tg.get(token) == 4:
            keep(token)
            tg.set(token, 0)
    keep("zsn")
    tg.set("zsn", 4)
    tg.send_gcode("G21 G90 G92 X0 Y0 Z0", check=True, allow="#186 work zero")
    yield
    if tg.stat() == tg.dialect.stat("ALARM"):
        tg.clear_alarm()
    _idle(tg)
    tg.drain()
    tg.send_gcode("G90 G1 Z0 F600", check=True, allow="#186 return")
    _idle(tg)


@pytest.mark.parametrize("wrapped", [False, True], ids=["plain", "json"])
def test_probe_runs_after_tool_change(tg, zmin_probe, record, wrapped):
    reports = []
    tool = 4 if wrapped else 3                          # a different tool per case, so a rejected change shows
    change = f"T{tool} M6"
    if wrapped:
        reply = tg.send_json({"gc": change}, allow="#186 tool change in JSON")
    else:
        reply = tg.send_gcode(change, allow="#186 tool change")
    assert reply.status == 0, tg.status_name(reply.status)
    assert _get(tg, "tool") == tool
    alarms = []
    for _ in range(2):
        resp = tg.send_gcode("G38.2 Z-1 F100", allow="#186 1 mm probe in free air")
        assert resp.status == 0, tg.status_name(resp.status)
        reports.append(_wait_prb(tg))
        _idle(tg)
        alarms.append(tg.stat() == tg.dialect.stat("ALARM"))     # wiki TinyG-Alarms: a failed probe alarms (250)
        if alarms[-1]:
            tg.clear_alarm()
        tg.send_gcode("G90 G1 Z0 F600", check=True, allow="#186 back to Z0 for the next probe")
        _idle(tg)
    record("i186", {"wrapped": wrapped, "reports": reports, "alarmed": alarms, "tool": _get(tg, "tool")})
    assert all(r.get("e") == 0 for r in reports), reports
    assert all(abs(r.get("z", 0) - (-1.0)) <= 0.002 for r in reports), reports   # each probe travelled the full 1 mm
    assert all(alarms), alarms                                                    # documented probe-failed alarm (250)
    assert _get(tg, "posz") == pytest.approx(0.0, abs=0.001)
    assert _get(tg, "tool") == tool
    assert tg.stat() in tg.dialect.idle_stats
