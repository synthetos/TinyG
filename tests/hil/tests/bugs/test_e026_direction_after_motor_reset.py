"""E026 (GitHub #235, #267, #216): re-setting a motor's settings between moves must not leave the DIR pin stale.

Master 440.x st_reset() clears the cached prev_direction without driving the DIR pin, so after a host re-sends a
motor setting ($Ntr, $Nsa, $Nmi, $Npo, as senders do before homing) the next move in the other direction believes the
pin is already set and moves the wrong way (#267 root cause; #235 "$2tr change reverses Y"; #216 "second homing runs
the wrong way"). Edge keeps the cache consistent with the pin: stepper.c stepper_reset() "do not initialize
prev_direction", and prev_direction is written only together with the DIR bit in _load_move (stepper.c:1085-1091).

The stock reset sets the cached direction to STEP_INITIAL_DIRECTION (DIRECTION_CW = 0) and leaves the pin alone, so the
bug shows only when the pin is at level 1 when the reset happens and the next move needs level 0: the loader then sees
"no change" and never drives the pin. The test builds exactly that case for the axis's motors:
1. a preparation move toward the side whose DIR level is 1 (negative travel with $Npo=0, positive with $Npo=1), then a
   check AT REST, after the move's tail, that the pin really is 1 (the end of a move can load a sub-step segment that
   sets DIR to the other level with no pulse, MSO 2026-11-16; the preparation is retried with other lengths until the
   pin rests at 1);
2. the motor reset (re-setting the motor's travel-per-revolution to its own value), with the pin still 1;
3. a challenge move in the other direction, which needs level 0, with the pin read through the Atmel-ICE DURING the
   move (about 0.8 s into a ~2 s move, machine state RUN before and after the read). It must be 0
   (stepper.c: positive travel drives DIRECTION_CW ^ $Npo; override_steps.dir_level).
On stock master the challenge read stays 1 and the test fails; on edge it is 0. The settings value is unchanged, so
the bench geometry is untouched.
"""
import time

import pytest

from tinyg_hil.override_steps import dir_level

pytestmark = [pytest.mark.ice, pytest.mark.motion, pytest.mark.persists, pytest.mark.timeout(120),
              pytest.mark.bug("E026", on=["d440"])]

PORTS = {1: "A", 2: "F", 3: "E", 4: "D"}     # hardware.h PORT_MOTOR_1..4
DIR_BIT = 1                                  # hardware.h DIRECTION_BIT_bp
MOVE, FEED, READ_AT = 10.0, 300, 0.8          # ~2 s moves; read the pin while moving


def _dir_pin(ice, motor):
    return (ice.port_out(PORTS[motor]) >> DIR_BIT) & 1


def _move(tg, ice, motors, dist):
    """Run a relative X move; return the DIR pin levels read while it moves, and the machine state around the read."""
    tg.send_gcode(f"G91 G1 X{dist:.3f} F{FEED}", check=True, allow="E026 direction move")
    time.sleep(READ_AT)
    before = tg.stat()
    levels = {m: _dir_pin(ice, m) for m in motors}
    after = tg.stat()
    tg.wait_idle(timeout=30)
    return {"stat_before_read": before, "stat_after_read": after, "dir": levels}


def _rest(tg, ice, motors, dist):
    """Run a relative X move to completion and return the DIR pin levels at rest, after the move's tail."""
    tg.send_gcode(f"G91 G1 X{dist:.3f} F{FEED}", check=True, allow="E026 preparation move")
    tg.wait_idle(timeout=30)
    time.sleep(0.3)
    return {m: _dir_pin(ice, m) for m in motors}


def test_dir_pin_follows_travel_after_motor_reset(tg, machine, ice, keep, record):
    motors = machine.motors_for("x")
    if not motors:
        pytest.skip("no motor mapped to X")
    pol = {m: int(tg.get(f"{m}po")) for m in motors}
    if len(set(pol.values())) != 1:
        pytest.skip("X motors with different polarities: no single preparation leaves every pin at 1")
    keep(*(f"{m}tr" for m in motors))
    prep_sign = 1 if next(iter(pol.values())) else -1            # the travel direction whose DIR level is 1
    tg.send_gcode("G21 G90", check=True, allow="units")
    tg.send_gcode("G28.3 X0", check=True, allow="E026 declare X")
    seen = {"polarity": pol, "preparations": []}
    try:
        for length in (MOVE, 9.0, 7.75, 6.0):                    # until the pin rests at 1 after the tail
            rest = _rest(tg, ice, motors, prep_sign * length)
            seen["preparations"].append({"length": prep_sign * length, "rest": rest})
            if all(v == 1 for v in rest.values()):
                break
        else:
            pytest.fail(f"could not leave the DIR pin at 1 at rest: {seen}")
        for m in motors:                                          # same value: only the motor reset runs
            tg.set(f"{m}tr", tg.get(f"{m}tr"), allow="E026 re-send motor setting")
        seen["after_reset"] = {m: _dir_pin(ice, m) for m in motors}
        seen["challenge"] = _move(tg, ice, motors, -prep_sign * MOVE)
    finally:
        tg.send_gcode("G90", check=True, allow="absolute")
    record("e026", seen)
    run = tg.dialect.stats("RUN")
    assert all(v == 1 for v in seen["after_reset"].values()), seen      # the reset left the pin at 1
    assert seen["challenge"]["stat_before_read"] in run and seen["challenge"]["stat_after_read"] in run, seen
    for m in motors:      # stock: cache 0 == wanted 0, pin never driven, stays 1
        assert seen["challenge"]["dir"][m] == dir_level(prep_sign < 0, pol[m]) == 0, (m, seen)
