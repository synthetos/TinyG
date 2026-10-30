"""E124: on 449.01 the canned $test programs 4 (arcs), 6 (feedhold), 7 (M-codes) and 8 (JSON) end
in ALARM instead of running to the end of the file.

Per the audit row for each test: T4 test_004_arcs.h:28 `N160 g3x0y0z40i20` raises ALARM 159;
T6 test_006_feedhold.h:16 `g0x100v76.765` raises ALARM 131 for the V word; T7 test_007_Mcodes.h:59
`g3x0y0i10` raises ALARM 159; T8 runs every {gc:...} line ahead of the planner (E034) and line 32
raises ALARM 159. Programs only run in char mode on 449.01 (E123). Tests 9, 10 (hours), 13 and 14
(rewrite EEPROM) and 2 (needs homing switches) are deliberately not run.
"""
import time

import pytest

pytestmark = pytest.mark.dialect("d449")


def _enter_char_mode(tg):
    """Switch to char mode starting from fresh line-mode buffers.

    On 449.01 the first line after the switch is replaced by stale text from the base of the RX
    pool (E079). Right after a reset that text is the tokenized {"rxm":0} line itself, which only
    yields an error, so a blank line is spent on it.
    """
    tg.reset()
    tg.set("rxm", 0)
    tg.transport.write(b"\n")
    tg.drain(quiet=0.5)


def _run_canned(tg, n: int, timeout: float) -> list:
    """Start $test=n (char mode) and return every event up to and including the EOF report."""
    eof = tg.dialect.status("EOF")
    tg.send_json({"txt": f"$test={n}"}, check=True, allow=f"canned test {n}")
    events, end = [], time.monotonic() + timeout
    while True:
        ev = tg.transport.next_event(end - time.monotonic())
        assert ev is not None, f"no EOF report from $test={n} within {timeout}s"
        events.append(ev)
        if ev.kind == "exception" and ev.data["er"].get("st") == eof:
            return events


def _settle(tg, timeout=120.0) -> int:
    """Wait until the queued moves finish (idle) or the machine sits in ALARM; return stat."""
    end = time.monotonic() + timeout
    while True:
        stat = tg.stat()
        if stat == tg.dialect.stat("ALARM"):
            return stat
        if stat in tg.dialect.idle_stats and tg.get("qr") == tg.planner_buffers:
            return stat
        assert time.monotonic() < end, f"still busy after {timeout}s (stat {stat})"
        time.sleep(0.2)


def _check_runs_clean(tg, n):
    events = _run_canned(tg, n, timeout=240)
    stat = _settle(tg)
    eof = tg.dialect.status("EOF")
    errors = [e.raw for e in events + tg.drain(quiet=0.3)
              if e.kind == "malformed" or (e.kind == "exception" and e.data["er"].get("st") != eof)]
    assert not errors, errors
    assert stat in tg.dialect.idle_stats


@pytest.mark.motion
@pytest.mark.persists
@pytest.mark.resets_board
@pytest.mark.timeout(420)
@pytest.mark.parametrize("n", [
    pytest.param(4, marks=pytest.mark.bug("E124", on=["d449"]), id="t4-arcs"),
    pytest.param(6, marks=pytest.mark.bug("E124", on=["d449"]), id="t6-feedhold"),
    pytest.param(7, marks=pytest.mark.bug("E124", on=["d449"]), id="t7-mcodes"),
    pytest.param(8, marks=pytest.mark.bug("E124", on=["d449"]), id="t8-json"),
])
def test_canned_test_runs_to_eof_without_alarm(tg, keep, n):
    keep("rxm")
    try:
        _enter_char_mode(tg)
        _check_runs_clean(tg, n)
    finally:
        tg.reset()      # clears ALARM, G92 and coordinate system; keep() then restores rxm
