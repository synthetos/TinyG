"""E026 (GitHub #235, #267, #216): re-setting a motor's settings between moves must not leave the DIR pin stale.

Master 440.x st_reset() clears the cached prev_direction without driving the DIR pin, so after a host re-sends a
motor setting ($Ntr, $Nsa, $Nmi, $Npo, as senders do before homing) the next move in the other direction believes the
pin is already set and moves the wrong way (#267 root cause; #235 "$2tr change reverses Y"; #216 "second homing runs
the wrong way"). Edge keeps the cache consistent with the pin: stepper.c stepper_reset() "do not initialize
prev_direction", and prev_direction is written only together with the DIR bit in _load_move (stepper.c:1085-1091).

The test reads the real DIR output pin through the Atmel-ICE at rest (PORTx.OUT bit DIRECTION_BIT_bp, hardware.h),
never during motion: after a positive move, after re-setting the motor's travel-per-revolution to its own value (the
setter runs the motor reset), and after the negative move that follows.
The check is relative: the pin level after the negative move must differ from the level after the positive move,
and return to it after the next positive move (absolute levels depend on the driver wiring and $Npo; they are
recorded). The settings value is unchanged, so the bench geometry is untouched.
"""
import pytest

pytestmark = [pytest.mark.ice, pytest.mark.motion, pytest.mark.persists, pytest.mark.timeout(120),
              pytest.mark.bug("E026", on=["d440"])]

PORTS = {1: "A", 2: "F", 3: "E", 4: "D"}     # hardware.h PORT_MOTOR_1..4
DIR_BIT = 1                                  # hardware.h DIRECTION_BIT_bp
MOVE = 3.0


def _dir_pin(ice, motor):
    return (ice.port_out(PORTS[motor]) >> DIR_BIT) & 1


def _move(tg, dist):
    line = f"G91 G1 X{dist:.3f} F600"
    tg.send_gcode(line, check=True, allow="E026 direction move")
    tg.wait_idle(timeout=30)


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
        _move(tg, MOVE)
        seen["after_positive"] = {m: _dir_pin(ice, m) for m in motors}
        for m in motors:                                      # same value: only the motor reset runs
            tg.set(f"{m}tr", tg.get(f"{m}tr"), allow="E026 re-send motor setting")
        seen["after_reset"] = {m: _dir_pin(ice, m) for m in motors}
        _move(tg, -MOVE)
        seen["after_negative"] = {m: _dir_pin(ice, m) for m in motors}
        _move(tg, MOVE)                                       # and back again, the #216 'second homing' order
        seen["after_second_positive"] = {m: _dir_pin(ice, m) for m in motors}
    finally:
        tg.send_gcode("G90", check=True, allow="absolute")
    record("e026", {"polarity": pol, "dir_pin": seen})
    for m in motors:      # a stale pin (the bug) keeps the positive level through the reversal
        assert seen["after_negative"][m] != seen["after_positive"][m], seen
        assert seen["after_second_positive"][m] == seen["after_positive"][m], seen
