"""Diagnostic: where the CPU goes while idle, moving, and holding (statistical PC sampling over the ICE).

Feeds the race/starvation bugs: how much time the step, loader and exec ISRs take during motion
(window size for E114/E116/E119/E120/E115) and whether the main loop starves.
"""
import time

import pytest

from tinyg_hil.profile import Profiler

pytestmark = [pytest.mark.diagnostic, pytest.mark.ice, pytest.mark.motion]
MOVE = "long slow X move for profiling, then back"


def test_profile_idle_motion_and_hold(tg, ice, record):
    prof = Profiler(ice)
    record("idle", Profiler.summary(prof.sample(150)))
    tg.send_gcode("G91 G1 X10 F300", check=True, allow=MOVE)          # ~2 s of motion
    time.sleep(0.3)
    record("moving", Profiler.summary(prof.sample(150)))
    tg.send_raw(b"!")
    time.sleep(0.5)
    record("holding", Profiler.summary(prof.sample(100)))
    tg.send_raw(b"~")
    tg.wait_idle(timeout=30)
    tg.send_gcode("G91 G1 X-10 F600", allow=MOVE)
    tg.wait_idle(timeout=30)
