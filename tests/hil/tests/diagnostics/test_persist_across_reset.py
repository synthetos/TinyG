"""Diagnostic: do motor geometry settings written over serial survive a soft reset?

On 2026-10-30 every profile that reset the board read motor 1-4 sa/tr/mi as 0 right after the reset,
while sessions' snapshots (RAM) were good. write_persistent_value() refuses to save while
cm.cycle_state != CYCLE_OFF and only reports an asynchronous exception, so a stuck cycle state would
leave RAM and EEPROM apart. This records the cycle state (ICE), every asynchronous line around the
writes, and the values read back after a reset.
"""
import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.persists, pytest.mark.resets_board]

TOKENS = [f"{m}{t}" for m in range(1, 5) for t in ("sa", "tr", "mi")]


def test_geometry_survives_reset(tg, record, request):
    ice = request.getfixturevalue("ice") if request.config.getoption("--ice") else None
    before = {t: tg.get(t) for t in TOKENS}
    record("before", before)
    if ice is not None:
        record("cycle_state_before", ice.peek("cm.cycle_state"))
        record("machine_state_before", ice.peek("cm.machine_state"))
    events = []
    # write a different value and back, so both writes really reach write_persistent_value()
    for value in (before["1tr"] + 1, before["1tr"]):
        events += tg.send_json({"1tr": value}).events
    for t, v in before.items():
        events += tg.send_json({t: v}).events
    record("async_during_writes", [e.raw for e in events if e.kind in ("exception", "text", "malformed")])
    tg.reset()
    after = {t: tg.get(t) for t in TOKENS}
    record("after_reset", after)
    lost = {t: (before[t], after[t]) for t in TOKENS if not tg.values_equal(t, before[t], after[t])}
    assert not lost, f"settings lost across reset: {lost}"
