"""N114: after a stepper reset, a motor's step phase depends on which way it last moved.

stepper_reset() (stepper.c) zeroes every motor's substep_accumulator but keeps prev_direction and prev_step_sign. The
next segment that moves the motor (_load_move) mirrors the accumulator to -dda_ticks_X_substeps only if the travel
reversed; a motor that keeps its direction starts from 0 and steps on the first DDA tick. So after a reset:
  - a motor that keeps its direction emits a whole step early: a move of n + 0.2 steps gives n + 1 steps;
  - two motors on one axis (a dual gantry) whose last directions differ end up one step apart.
stepper_reset() runs on $Nmi/$Ntr/$Nsa/$Nma sets, on st_clc and on every alarm halt (mp_halt_runtime).

Seen on bench 3 in the night soak (Y motors M2 + M3 stepped 19026 and 19027, MSO capture 20261118T010200Z: M3 stepped
on the program's first tick, M2 a full step later) and reproduced through the ICE (test_bench3_twin_phase).

The tests count the firmware's own steps per motor (encoder_steps + steps_run, read through the ICE at rest):
  - test_first_move_after_reset: a motor that last moved forward gets a reset ($Nmi rewritten with its own value)
    and then a move of n + 0.2 steps forward, then n + 0.8: it must step n, then n + 1 (the nearest whole steps).
  - test_twins_after_remap: a second motor joins the first on Y with the same geometry. Both move +Y, then the
    second is remapped to Z, moved -Z alone and mapped back to Y, as when squaring a gantry by moving one side. A Y
    move of n + 0.2 steps must step both motors the same number of times.
Needs --ice.
"""
import time

import pytest

pytestmark = [pytest.mark.motion, pytest.mark.ice, pytest.mark.persists, pytest.mark.dialect("d449"),
              pytest.mark.timeout(180)]

WHY = "N114 step phase after a stepper reset"
GEOMETRY = ("sa", "tr", "mi", "po")


def _spm(tg, m):
    return 360.0 * tg.get(f"{m}mi") / (tg.get(f"{m}sa") * tg.get(f"{m}tr"))


def _count(tg, ice, motors):
    """The firmware's own step count per motor at rest (encoder_steps plus the last segment's steps_run)."""
    tg.wait_idle(timeout=60)
    time.sleep(0.05)
    v = ice.peek_many([f"en.en[{m - 1}].{f}" for m in motors for f in ("encoder_steps", "steps_run")])
    return {m: v[f"en.en[{m - 1}].encoder_steps"] + v[f"en.en[{m - 1}].steps_run"] for m in motors}


def _y(tg, dy, feed=400):
    tg.send_gcode(f"G91 G1 Y{dy:.5f} F{feed}", allow=WHY)
    tg.send_gcode("G90", allow=WHY)
    tg.wait_idle(timeout=60)                           # motor settings are refused while a move runs (status 117)


def _y_motor(tg):
    for m in (1, 2, 3, 4):
        if tg.get(f"{m}ma") == 1:
            return m
    pytest.skip("no motor on Y")


@pytest.mark.bug("N114", on=["d449"])
def test_first_move_after_reset(tg, ice, keep, record):
    m = _y_motor(tg)
    keep(f"{m}mi")
    spm = _spm(tg, m)
    n = max(20, round(0.3 * spm))
    tg.send_gcode("G21", allow=WHY)
    _y(tg, 0.5)
    _y(tg, -0.25)
    _y(tg, 0.1)                                        # the motor last moved forward
    got = []
    for frac in (0.2, 0.8):
        tg.set(f"{m}mi", tg.get(f"{m}mi"), allow=WHY)  # the setter calls stepper_reset(); nothing else changes
        c0 = _count(tg, ice, [m])
        _y(tg, (n + frac) / spm)
        c1 = _count(tg, ice, [m])
        got.append(c1[m] - c0[m])
    _y(tg, -(0.35 + (2 * n + 1.0) / spm))
    record("n114_first_move", {"motor": m, "steps_per_mm": spm, "n": n, "steps": got, "want": [n, n + 1]})
    assert got == [n, n + 1], f"moves of {n}.2 and {n}.8 steps after a stepper reset stepped {got}, want {[n, n + 1]}"


@pytest.mark.bug("N114", on=["d449"])
def test_twins_after_remap(tg, ice, keep, record):
    a = _y_motor(tg)
    b = next((m for m in (1, 2, 3, 4) if m != a and tg.get(f"{m}ma") == 1), None)
    if b is None:                                      # no twin on this bench: make one with the same geometry
        b = next(m for m in (4, 3, 2, 1) if m != a)
    keep(f"{b}ma", *(f"{b}{t}" for t in GEOMETRY))
    for t in GEOMETRY:
        tg.set(f"{b}{t}", tg.get(f"{a}{t}"), allow=WHY)
    tg.set(f"{b}ma", 1, allow=WHY)
    spm = _spm(tg, a)
    n = max(20, round(0.3 * spm))
    tg.send_gcode("G21", allow=WHY)
    _y(tg, 0.5)                                        # both twins move +Y
    tg.set(f"{b}ma", 2, allow=WHY)                     # square the gantry: one side alone on Z
    tg.send_gcode("G91 G1 Z-0.5 F300", allow=WHY)
    tg.send_gcode("G90", allow=WHY)
    tg.wait_idle(timeout=60)
    tg.set(f"{b}ma", 1, allow=WHY)                     # back on Y (the setter calls stepper_reset())
    c0 = _count(tg, ice, [a, b])
    _y(tg, (n + 0.2) / spm)
    c1 = _count(tg, ice, [a, b])
    ran = {m: c1[m] - c0[m] for m in (a, b)}
    tg.send_gcode("G91 G1 Z0.5 F300", allow=WHY)
    tg.send_gcode("G90", allow=WHY)
    _y(tg, -(0.5 + (n + 0.2) / spm))
    record("n114_twins", {"motors": [a, b], "steps_per_mm": spm, "n": n, "steps": ran})
    assert ran[a] == ran[b], f"Y motors {a} and {b} stepped {ran[a]} and {ran[b]} for {n}.2 steps after the remap"
