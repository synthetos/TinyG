"""Diagnostic: does a lone M3 reach the spindle after a reset, and if not, where does it stop?

The v2 + edge-next candidate failed test_polarity_output_latches on bench 2 because cm.gm.spindle_mode
read 8 and 16 for M3 and M4: GCodeState_t is built with __BITFIELDS and the ICE peek returned the
whole byte. With bitfield decoding this records machine, cycle and hold state and the runtime block
state around M3/M4/M5, before and after a soft reset, and checks the spindle mode reads 1 and 2.
"""
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.ice, pytest.mark.resets_board]

SYMBOLS = ["cm.gm.spindle_mode", "cm.machine_state", "cm.cycle_state", "cm.hold_state",
           "cm.motion_state", "mr.block_state"]


def _state(ice):
    out = {}
    for s in SYMBOLS:
        try:
            out[s] = ice.peek(s)
        except Exception as exc:  # noqa: BLE001 - record what could not be read
            out[s] = f"unreadable: {exc}"
    return out


def _trace(tg, ice, line, seconds=3.0):
    reply = tg.send_gcode(line)
    samples = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        samples.append(_state(ice))
        time.sleep(0.25)
    return {"line": line, "reply": getattr(reply, "raw", str(reply)), "stat": tg.get("stat"),
            "samples": samples[:2] + samples[-1:]}


def test_lone_m3_executes(tg, ice, record):
    if not tg.guard.allow_motion:
        pytest.skip("requires --allow-motion on a bare board: changes spindle output levels")
    record("before_reset", [_trace(tg, ice, line) for line in ("M5", "M3", "M5")])
    tg.reset()
    record("after_reset_state", _state(ice))
    after = [_trace(tg, ice, line) for line in ("M5", "M3", "M4", "M5")]
    record("after_reset", after)
    modes = [t["samples"][-1]["cm.gm.spindle_mode"] for t in after]
    assert modes == [0, 1, 2, 0], f"spindle modes after M5/M3/M4/M5: {modes}"
