"""Timing gate E116: a feedhold that arrives while the last block finishes must not stick.

E116: a '!' landing at the final-segment handoff left hold REQUESTED/SYNC, the cycle never ended, a
later '!' was ignored and the next move was held immediately. The move's first-to-last step span is
measured on the Saleae; each trial writes the move, sleeps the span plus an offset on the host and
sends '!'. The offset is a host sleep target: USB and scheduling jitter and the write-to-first-step
latency move the real arrival, so trials are swept in 5 ms steps until several land after the move
ended, then twice in 0.3 ms steps across that boundary, so some land on the final segment.

The verdict is a coarse logical outcome, not a measured ISR window: 0.6 s after each '!' the hold must
be HOLD (it engaged) or OFF (the move had ended), never left REQUESTED/SYNC; both outcomes must be
seen; then '~' and a second move must run, the analyzer counts the steps of both moves, and the
machine must end idle with no hold. Every capture (.sal) and its raw edges are kept in the run
directory, with host timing markers per trial in the record.
"""
import json
from pathlib import Path
import time

import pytest

from tinyg_hil import benches, saleae

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.dialect("d449")]

LENGTH = 2.0            # mm
FEED = 600.0            # mm/min
OFFSETS = [i * 0.005 for i in range(-4, 41)]   # s after the measured move duration, until it has ended
FINISHED_RUN = 4                                 # stop after this many trials where the move had already ended
MOVE = "2 mm +X moves with a feedhold swept across the end, then back"


def _bench(tg):
    config = benches.load()
    bench = config.get(tg.transport.port) or {}
    if not bench.get("saleae"):
        pytest.skip(f"no Saleae recorded for {tg.transport.port}")
    chan = {v: int(k) for k, v in bench["saleae_channels"].items()}
    if "M1 step" not in chan:
        pytest.skip("M1 step is not on the analyzer")
    return bench, chan["M1 step"]


@pytest.mark.timeout(1500)
def test_late_feedhold_never_sticks(tg, record, request):
    bench, step = _bench(tg)
    recorder = request.config._tinyg_recorder
    assert recorder, "run records required: the captures are the evidence"
    out = recorder.dir.resolve()
    assert tg.get("1ma") == 0, "X must be motor 1"
    spm = 360.0 / tg.get("1sa") * tg.get("1mi") / tg.get("1tr")
    feed = min(FEED, tg.get("xvm"), tg.get("xfr"))
    per_move = round(LENGTH * spm)
    hold_off = tg.dialect.hold("OFF")
    tg.send_gcode("G21 G91 G94", check=True)
    tg.wait_idle()
    # first-to-last step span of one move (not command-to-last step: the write-to-first-step latency is not in it)
    with saleae.Capture(bench["saleae"], channels=[step], seconds=None, save_as=out / "e116-span") as cap:
        time.sleep(0.2)
        tg.send_gcode(f"G1 X{LENGTH:g} F{feed:g}", check=True, allow=MOVE)
        tg.wait_idle(timeout=30)
        time.sleep(0.2)
    rises = saleae.rising(cap.edges()[step])
    tg.send_gcode(f"G1 X{-LENGTH:g} F{feed:g}", allow=MOVE)             # return the calibration move
    tg.wait_idle(timeout=30)
    assert abs(len(rises) - per_move) <= 1, f"{len(rises)} pulses, expected {per_move}"
    duration = rises[-1] - rises[0]
    trials, bad, finished = [], [], 0

    def trial(offset):
        name = f"e116-trial-{len(trials):03d}"
        with saleae.Capture(bench["saleae"], channels=[step], seconds=None, save_as=out / name) as cap:
            started = time.monotonic()
            time.sleep(0.2)
            written = time.monotonic()
            tg.transport.write(f"G1 X{LENGTH:g} F{feed:g}\n".encode())
            time.sleep(max(0.0, duration + offset))
            banged = time.monotonic()
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
        trial_rises = saleae.rising(cap.edges()[step])
        pulses = len(trial_rises)
        state = {"hold": tg.get("hold"), "cycs": tg.get("cycs")}
        # host clock markers (not instrument time): capture start -> write -> '!' as the host issued them
        result = {"capture": name + ".sal", "host_sleep_target_after_span_s": round(offset, 4),
                  "host_write_after_capture_start_s": round(written - started, 6),
                  "host_bang_after_write_s": round(banged - written, 6),
                  "first_step_s": trial_rises[0] if trial_rises else None,
                  "first_move_last_step_s": trial_rises[per_move - 1] if pulses >= per_move else None,
                  "hold_after_bang": hold_after, "cycs_after_bang": cycs_after,
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
        if boundary is not None:                             # fine: 0.3 ms steps, 6 ms before to 3 ms after, twice
            for _ in range(2):
                for i in range(31):
                    trial(boundary - 0.006 + i * 0.0003)
    finally:
        record("e116_sweep", {"first_to_last_step_s": duration, "span_capture": "e116-span.sal",
                              "sample_rate": cap.rate, "step_channel": step, "steps_per_mm": spm,
                              "verdict_kind": "logical outcome per trial; offsets are host sleep targets", "trials": trials,
                              "boundary_reached": finished >= FINISHED_RUN,
                              "held_then_finished": sorted({t["hold_after_bang"] for t in trials})})
        if tg.get("hold") != hold_off:
            tg.send_raw(b"~")
        tg.send_gcode("G90")
    assert finished >= FINISHED_RUN, "the sweep never reached the end of the move"
    seen = {t["hold_after_bang"] for t in trials}
    assert {hold_off, tg.dialect.hold("HOLD")} <= seen, f"sweep did not cover both outcomes: {sorted(seen)}"
    assert not bad, f"stuck or lost motion: {bad}"
