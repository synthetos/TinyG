"""Diagnostic (E143 candidate): does each move lose a step, or is one step lost once?

The step-counter test sees 1 mm of X produce spu-1 pulses (9 vs 10, 639 vs 640) on stock and
edge-next alike. Five consecutive +1 mm moves, then five -1 mm moves, recording per move: the
firmware's physical step count (test_hook.steps, never rewritten; test-hooks build), the encoder count
and the runtime following error.
"""
import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.ice, pytest.mark.motion]


@pytest.mark.matrix("motor-geometry")
def test_step_count_per_move(tg, machine, ice, record):
    motors = machine.motors_for("x")
    if not motors:
        pytest.skip("no motor mapped to X")
    m = motors[0] - 1
    spu = machine.steps_per_unit("x")
    if "test_hook" not in ice.symbols.sym:
        pytest.skip("needs the test-hooks build (physical step count)")

    def snap():
        v = ice.peek_many([f"test_hook.steps[{m}]", f"en.en[{m}].encoder_steps", f"mr.following_error[{m}]",
                           "mr.position[0]"])
        return list(v.values())

    rows, prev = [], snap()
    for direction in (1, -1):
        for _ in range(5):
            tg.send_gcode(f"G91 G1 X{direction} F120", check=True, allow="1 mm X step-count move")
            tg.wait_idle()
            now = snap()
            rows.append({"dir": direction, "physical": now[0] - prev[0], "encoder": now[1] - prev[1],
                         "following_error": round(now[2], 3), "mr_x": round(now[3], 4)})
            prev = now
    record("spu", spu)
    record("moves", rows)
    record("net_physical", sum(r["physical"] for r in rows))
