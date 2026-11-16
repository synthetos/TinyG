"""E026 (GitHub #235, #267, #216): re-setting a motor's settings between moves must not leave the DIR pin stale.

Master 440.x st_reset() clears the cached prev_direction without driving the DIR pin, so after a host re-sends a
motor setting ($Ntr, $Nsa, $Nmi, $Npo, as senders do before homing) the next move in the other direction believes the
pin is already set and moves the wrong way (#267 root cause; #235 "$2tr change reverses Y"; #216 "second homing runs
the wrong way"). Edge keeps the cache consistent with the pin: stepper.c stepper_reset() "do not initialize
prev_direction", and prev_direction is written only together with the DIR bit in _load_move (stepper.c:1085-1091).

The test reads the real DIR output pin through the Atmel-ICE (PORTx.OUT bit DIRECTION_BIT_bp, hardware.h) DURING
each move (one read, about 0.8 s into a ~2 s move, machine state RUN): a positive move, a re-set of the motor's
travel-per-revolution to its own value (the setter runs the motor reset), a negative move, a second reset, and a
positive move. Resetting after both directions guarantees that one reversal starts from a cache that disagrees with
the pin on master, whichever DIR level a direction maps to. The level must match the firmware's mapping for each
direction (stepper.c: positive travel drives DIRECTION_CW ^ $Npo, i.e. override_steps.dir_level).
Reading at rest is not valid: the end of some moves loads a sub-step segment that sets DIR to the reverse level with
no step pulse (MSO, 2026-11-16, handoffs/2026-11-16-dir-flip-without-step.md), which made an earlier version of this
test read the reverse of the travel. The settings value is unchanged, so the bench geometry is untouched.
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
    """Run a relative X move; return the DIR pin levels and machine state read while it moves."""
    tg.send_gcode(f"G91 G1 X{dist:.3f} F{FEED}", check=True, allow="E026 direction move")
    time.sleep(READ_AT)
    stat = tg.stat()
    levels = {m: _dir_pin(ice, m) for m in motors}
    tg.wait_idle(timeout=30)
    return {"stat_at_read": stat, "dir": levels}


def test_dir_pin_follows_travel_after_motor_reset(tg, machine, ice, keep, record):
    motors = machine.motors_for("x")
    if not motors:
        pytest.skip("no motor mapped to X")
    keep(*(f"{m}tr" for m in motors))
    pol = {m: int(tg.get(f"{m}po")) for m in motors}
    tg.send_gcode("G21 G90", check=True, allow="units")
    tg.send_gcode("G28.3 X0", check=True, allow="E026 declare X")
    seen = {}
    try:
        seen["positive"] = _move(tg, ice, motors, MOVE)
        for m in motors:                                      # same value: only the motor reset runs
            tg.set(f"{m}tr", tg.get(f"{m}tr"), allow="E026 re-send motor setting")
        seen["after_reset"] = {m: _dir_pin(ice, m) for m in motors}
        seen["negative"] = _move(tg, ice, motors, -MOVE)
        for m in motors:                                      # reset again after the other direction, so one of
            tg.set(f"{m}tr", tg.get(f"{m}tr"), allow="E026 re-send motor setting")   # the two reversals meets a stale cache
        seen["after_second_reset"] = {m: _dir_pin(ice, m) for m in motors}
        seen["second_positive"] = _move(tg, ice, motors, MOVE)   # and back again, the #216 'second homing' order
    finally:
        tg.send_gcode("G90", check=True, allow="absolute")
    record("e026", {"polarity": pol, "dir_pin": seen})
    run = tg.dialect.stats("RUN")
    for key, positive in (("positive", True), ("negative", False), ("second_positive", True)):
        assert seen[key]["stat_at_read"] in run, (key, seen)          # the read happened during the move
        for m in motors:      # a stale pin (the bug) keeps the previous level through the reversal
            assert seen[key]["dir"][m] == dir_level(positive, pol[m]), (key, m, seen)
