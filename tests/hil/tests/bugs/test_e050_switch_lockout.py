"""E050: a release during lockout must not disarm the next limit closure.

Timing-pending: ICE/test hooks establish logic outcomes, not real lockout windows,
ISR latency or race timing. Scope/logic-analyzer confirmation on a normal build
is still required; the deterministic host regression is the primary logic proof.

Requires a TEST_HOOKS=1 build and --ice on an unwired board. The 50 ms case
uses the production lockout; 250 ms provides a wider debugger stimulus window.
Firmware tick observations retain the original closure timer so debugger latency
cannot turn a release-rearmed timer into passing evidence. Five invalid trials
skip explicitly; proven logic failures are not retried. No motion is commanded.
"""
import time

import pytest

from tinyg_hil.driver import NoResponse
from tinyg_hil.switch_window import trials

pytestmark = [pytest.mark.timing, pytest.mark.ice, pytest.mark.dialect("d449"),
              pytest.mark.profile("switches-no")]


def _wait(tg, predicate, timeout=3):
    deadline = time.monotonic() + timeout
    value = None
    while time.monotonic() < deadline:
        try:
            value = tg.stat()
        except NoResponse:          # a query that arrived while the ICE held the CPU is lost: ask again
            continue
        if predicate(value):
            return value
        time.sleep(0.02)
    pytest.fail(f"machine status did not reach expected condition; last={value}")


@pytest.mark.bug("E050", on=[])
@pytest.mark.parametrize("lockout_ms", [50, 250], ids=["production-50ms", "wide-250ms"])
def test_release_during_lockout_rearms_limit(tg, keep, switches, ice, record, lockout_ms):
    record("validation_status", "timing-pending")
    if "test_hook" not in ice.symbols.sym:
        pytest.skip("requires TEST_HOOKS=1; halted pin writes do not exercise port edges")
    keep("xsn", "xsx")
    tg.set("xsn", 2, allow="enable X-min limit for no-motion switch test")
    tg.set("xsx", 0, allow="disable unused X-max input")
    switches.open("x", "min")
    time.sleep(0.1)
    original = ice.peek("sw.s[0].lockout_ms")
    ice.poke("sw.s[0].lockout_ms", lockout_ms)
    try:
        def sample():
            with ice.halted():
                values = ice.peek_many(["rtc.sys_ticks", "sw.s[0].timeout._start",
                                        "sw.s[0].timeout._delay", "sw.s[0].state"])
                pin = ice.switch_pin("x", "min")
            return dict(ticks=values["rtc.sys_ticks"], start=values["sw.s[0].timeout._start"],
                        delay=values["sw.s[0].timeout._delay"], state=values["sw.s[0].state"], raw_pin=pin)

        def prepare():
            switches.open("x", "min")
            # Allow both the old closure lockout and a deferred release's new
            # lockout to expire before trying again. No board reset is needed.
            time.sleep(2 * lockout_ms / 1000 + .15)
            tg.clear_alarm()
            _wait(tg, lambda state: state != tg.dialect.stat("ALARM"))

        trial = trials(sample, lambda: switches.close("x", "min"),
                       lambda: switches.open("x", "min"), prepare, record, lockout_ms)
        if trial is None:
            pytest.skip("stimulus window not achieved: 5 invalid trials; see stimulus_trials")
        record("pulse_window", trial)
        _wait(tg, lambda state: state == tg.dialect.stat("ALARM"))
        tg.clear_alarm()
        _wait(tg, lambda state: state != tg.dialect.stat("ALARM"))
        time.sleep(lockout_ms / 1000 + 0.15)
        released = ice.peek("sw.s[0].state")
        record("cached_state_after_release", released)
        assert released == 0, "E050: cached input stayed active after the ignored release"
        switches.close("x", "min")
        _wait(tg, lambda state: state == tg.dialect.stat("ALARM"))
        record("second_closure_alarmed", True)
    finally:
        switches.open("x", "min")
        time.sleep(lockout_ms / 1000 + 0.15)
        tg.clear_alarm()
        ice.poke("sw.s[0].lockout_ms", original)
