"""Timing gate E116: a feedhold that arrives while the last block finishes must not stick.

E116: a '!' landing at the final-segment handoff left hold REQUESTED/SYNC, the cycle never ended, a
later '!' was ignored and the next move was held immediately. The first move's duration is measured
on the Saleae, then '!' is swept later in 5 ms steps until several trials land after the move ended,
so some land on the final segment. 0.6 s after each '!' the hold must be HOLD (it engaged) or
OFF (the move had ended), never left REQUESTED/SYNC; then '~' and a second move must run, the analyzer
counts the steps of both moves, and the machine must end idle with no hold.
"""
import json
from pathlib import Path
import time

import pytest

from tinyg_hil import saleae

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.dialect("d449")]

LENGTH = 2.0            # mm
FEED = 600.0            # mm/min
OFFSETS = [i * 0.005 for i in range(-4, 41)]   # s after the measured move duration, until it has ended
FINISHED_RUN = 4                                 # stop after this many trials where the move had already ended
MOVE = "2 mm +X moves with a feedhold swept across the end, then back"


def _bench(tg):
    config = json.loads((Path(__file__).resolve().parents[2] / "config/benches.json").read_text())
    bench = config.get(tg.transport.port) or {}
    if not bench.get("saleae"):
        pytest.skip(f"no Saleae recorded for {tg.transport.port}")
    chan = {v: int(k) for k, v in bench["saleae_channels"].items()}
    if "M1 step" not in chan:
        pytest.skip("M1 step is not on the analyzer")
    return bench, chan["M1 step"]


@pytest.mark.timeout(900)
def test_late_feedhold_never_sticks(tg, record):
    bench, step = _bench(tg)
    assert tg.get("1ma") == 0, "X must be motor 1"
    spm = 360.0 / tg.get("1sa") * tg.get("1mi") / tg.get("1tr")
    feed = min(FEED, tg.get("xvm"), tg.get("xfr"))
    per_move = round(LENGTH * spm)
    hold_off = tg.dialect.hold("OFF")
    tg.send_gcode("G21 G91 G94", check=True)
    tg.wait_idle()
    # duration of one move, command written to last step pulse
    with saleae.Capture(bench["saleae"], channels=[step], seconds=None) as cap:
        time.sleep(0.2)
        tg.send_gcode(f"G1 X{LENGTH:g} F{feed:g}", check=True, allow=MOVE)
        tg.wait_idle(timeout=30)
        time.sleep(0.2)
    rises = saleae.rising(cap.edges()[step])
    assert abs(len(rises) - per_move) <= 1, f"{len(rises)} pulses, expected {per_move}"
    duration = rises[-1] - rises[0]
    trials, bad, finished = [], [], 0

    def trial(offset):
        with saleae.Capture(bench["saleae"], channels=[step], seconds=None) as cap:
            time.sleep(0.2)
            tg.transport.write(f"G1 X{LENGTH:g} F{feed:g}\n".encode())
            time.sleep(max(0.0, duration + offset))
            tg.send_raw(b"!")
            tg.wait_for(lambda e: e.kind == "response", 5.0)    # the G1's reply, before any getter
            time.sleep(0.6)                                  # a real hold has stopped by now
            hold_after = tg.get("hold")
            cycs_after = tg.get("cycs")
            tg.send_raw(b"~")
            time.sleep(0.3)
            tg.send_gcode(f"G1 X{LENGTH:g} F{feed:g}", check=True, allow=MOVE)
            tg.wait_idle(timeout=30)
            time.sleep(0.2)
        pulses = len(saleae.rising(cap.edges()[step]))
        state = {"hold": tg.get("hold"), "cycs": tg.get("cycs")}
        result = {"offset_s": round(offset, 4), "hold_after_bang": hold_after, "cycs_after_bang": cycs_after,
                  "pulses": pulses, "expected": 2 * per_move, **state}
        trials.append(result)
        # 0.6 s after '!' the hold has either completed (HOLD) or never started (OFF, the move had ended);
        # REQUESTED/SYNC with nothing moving is the stuck request E116 describes
        stuck = hold_after not in (hold_off, tg.dialect.hold("HOLD"))
        if stuck or abs(pulses - 2 * per_move) > 2 or state["hold"] != hold_off or state["cycs"] != 0:
            bad.append(result)
        tg.send_gcode(f"G1 X{-2 * LENGTH:g} F{feed:g}", allow=MOVE)
        tg.wait_idle(timeout=30)
        return hold_after

    try:
        boundary = None
        for offset in OFFSETS:                               # coarse: 5 ms steps until the move has ended
            if trial(offset) == hold_off:
                finished += 1
                boundary = offset if boundary is None else boundary
                if finished >= FINISHED_RUN:
                    break
            else:
                finished, boundary = 0, None
        if boundary is not None:                             # fine: 0.5 ms steps over the 10 ms before it
            for i in range(21):
                trial(boundary - 0.010 + i * 0.0005)
    finally:
        record("e116_sweep", {"move_duration_s": duration, "steps_per_mm": spm, "trials": trials,
                              "boundary_reached": finished >= FINISHED_RUN,
                              "held_then_finished": sorted({t["hold_after_bang"] for t in trials})})
        if tg.get("hold") != hold_off:
            tg.send_raw(b"~")
        tg.send_gcode("G90")
    assert finished >= FINISHED_RUN, "the sweep never reached the end of the move"
    assert not bad, f"stuck or lost motion: {bad}"
