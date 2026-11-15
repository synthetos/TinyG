"""Offline: the bench 3 scenario generator never puts two consecutive spindle levels in the ambiguous band where the
program list merges them but the measured plateaus may not (seed 561469437 on 2026-11-14: duty 0.1309 then 0.1250)."""
import importlib.util
import random
from pathlib import Path

_spec = importlib.util.spec_from_file_location(
    "bench3_scenario", Path(__file__).parent / "diagnostics" / "test_bench3_scenario.py")
scenario = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(scenario)

# Bench 3 defaults: S 1000..2000 -> duty 0.125..0.200, off duty 0.1 (S1079 -> 0.1309, S1000 -> 0.1250).
PWM = dict(csl=1000, csh=2000, cpl=0.125, cph=0.2, wsl=1000, wsh=2000, wpl=0.125, wph=0.2, pof=0.1)
CAPS = {"x": 1200, "y": 1200, "z": 1200}


def test_consecutive_spindle_levels_are_same_or_distinct():
    band = (1e-9, 3 * scenario.DUTY_SAME)
    for seed in range(2000):
        prog = scenario._Program(random.Random(seed), CAPS, PWM)
        duties = [scenario._duty(PWM, m, s) for m, s in prog.spindle]
        for a, b in zip(duties, duties[1:]):
            step = abs(a - b)
            assert not (band[0] <= step < band[1]), f"seed {seed}: ambiguous step {a:.4f} -> {b:.4f}"
