"""N82: remapping a motor ($Nma) at rest must not make the next move jump.

$Nsa/$Ntr/$Nmi go through _set_motor_steps_per_unit(), which calls stepper_reset() and so
mp_set_steps_to_runtime_position(): every motor's step position is re-derived from the runtime position. set_ma()
(config_app.c) only stores the new axis. A remapped motor keeps its old axis's step position, and the first move
asks it for the difference in one segment: a PANIC "step rate exceeds DDA" (st 20) once that is more steps than the
segment has DDA ticks, a lurch below that. Seen on bench 1 (2026-11-06): a soak job set 2ma=0 with X at 1.000 and
PANICked within 20 ms of its first line (wiki TinyG-Configuration-for-Firmware-Version-0.97, $1MA: the axis a motor
drives; nothing says a remap needs a reset).

With X at 10 and Y at 0, the motor on Y is mapped to X and X moves 0.002 mm. Pass: no PANIC, and X ends at 10.002.
"""
import time

import pytest

pytestmark = [pytest.mark.motion, pytest.mark.persists, pytest.mark.resets_board,
              pytest.mark.dialect("d449"), pytest.mark.timeout(120),
              pytest.mark.bug("N82", on=[])]


def test_remapped_motor_does_not_jump(tg, keep, record):
    ymotor = next((m for m in (1, 2, 3, 4) if tg.get(f"{m}ma") == 1), None)
    if ymotor is None:
        pytest.skip("no motor on Y")
    keep(f"{ymotor}ma")
    tg.reset()
    tg.send_gcode("G21 G90 G17 G94 G54", check=True)
    tg.send_gcode("G28.3 X10 Y0 Z0", check=True, allow="X at 10, Y at 0")
    tg.wait_idle(timeout=10)                               # G28.3 is queued: let it set the origin first
    tg.set(f"{ymotor}ma", 0, allow="remap the Y motor to X at rest")
    mark = len(tg.transport.history)
    tg.send_gcode("G1 X10.002 F300", check=True, allow="N82 tiny move")
    dead = set(tg.dialect.stats("SHUTDOWN", "PANIC"))
    t0, stat = time.monotonic(), None
    while time.monotonic() - t0 < 20:
        stat = tg.stat()
        if stat in dead or (stat in tg.dialect.idle_stats and tg.get("qr") == 32):
            break
        time.sleep(0.1)
    exc = [e.raw for e in tg.transport.history[mark:] if e.kind == "exception"]
    r = {"motor": ymotor, "stat": stat, "exceptions": exc, "posx": tg.get("posx")}
    record("n82", r)
    print("N82", r)
    if stat in dead:
        tg.reset()
    assert stat not in dead, f"motor {ymotor} remapped to X jumped on the next move: {exc}"
    assert r["posx"] == pytest.approx(10.002, abs=0.001), r
