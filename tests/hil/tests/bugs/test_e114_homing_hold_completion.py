"""E114 integration coverage: repeated homing through mounted software switches.

Timing-pending: ICE/test hooks establish logic outcomes, not real lockout windows,
ISR latency or race timing. Scope/logic-analyzer confirmation on a normal build
is still required; the deterministic host regression is the primary logic proof.

Requires TEST_HOOKS=1, --ice and --allow-motion on the bare board. This test
checks complete search/latch/backoff cycles, but does not force the narrow
PENDING-to-HOLD interleaving: a stock pass is not evidence against E114.
The deterministic ordering regression is tests/host/test_homing_hold_completion.py.
"""
import time

import pytest

pytestmark = [pytest.mark.timing, pytest.mark.ice, pytest.mark.motion, pytest.mark.resets_board,
              pytest.mark.dialect("d449"), pytest.mark.profile("switches-no")]


def test_repeated_homing_completes_after_switch_holds(tg, keep, switches, ice, record):
    record("validation_status", "timing-pending")
    if "test_hook" not in ice.symbols.sym:
        pytest.skip("requires TEST_HOOKS=1; halted pin writes do not exercise port edges")
    try:
        ice.symbols.resolve("test_hook.mount[0].min_steps")
    except KeyError:
        pytest.skip("requires physical-step mounts with extrema; rebuild TEST_HOOKS=1")
    settings = {"xsn": 1, "xsx": 2, "xtn": -2, "xtm": 2,
                "xsv": 120, "xlv": 60, "xlb": 1, "xzb": 0.2}
    # Keep an unused opposite limit configured: disabling it exposes separate E055.
    keep(*settings)
    for token, value in settings.items():
        tg.set(token, value, allow="small X homing envelope for software-switch test")
    # The 0.2 mm backoff and 0.1 mm assertion need finer resolution than a rotary motor mapped to X.
    for motor in range(1, 5):
        if tg.get(f"{motor}ma") == 0:
            tokens = [f"{motor}{suffix}" for suffix in ("sa", "mi", "tr")]
            keep(*tokens)
            for token, value in zip(tokens, (1.8, 8, 1.25)):
                tg.set(token, value, allow="1280 steps/mm for the mounted X homing test")
    completed = False
    rounds = []
    try:
        for iteration in range(3):
            switches.unmount_all()
            switches.open("x", "min")
            switches.open("x", "max")
            time.sleep(0.1)
            tg.send_gcode("G21 G90", check=True)
            tg.send_gcode("G28.3 X0", check=True, allow="seed bare-board X machine position")
            tg.wait_idle()  # the origin callback must finish before the mount frame is sampled
            switches.mount("x", "min", at=-0.5)
            assert ("x", "min") in switches.frames, "homing must use a physical-step mount"
            tg.send_gcode("G28.2 X0", check=True, allow="home X against software switch at -0.5 mm")
            # The mount drives real edges in the main loop. ICE sampling here can halt between a
            # pin change and its port interrupt. Observe over serial until homing has finished.
            tg.wait_idle(timeout=25)
            with ice.halted():
                snapshot = ice.peek_many(["cm.cycle_state", "cm.homing_state", "cm.homed[0]",
                                          "cm.hold_state", "mr.position[0]"])
                snapshot["mount"] = switches.mount_status("x", "min")
                snapshot["physical_position"] = switches.physical_position("x")
            switches.unmount_all()
            assert switches.error is None, f"switch follower failed: {switches.error!r}"
            mount = snapshot["mount"]
            result = {"iteration": iteration, "mount": mount, "final": snapshot,
                      "min_position": mount["min_position"], "max_position": mount["max_position"]}
            rounds.append(result)
            record("homing_rounds", rounds)
            assert mount["trips"] == 1, "homing did not cross the switch exactly once"
            assert not mount["closed"], "homing did not release the physical switch"
            assert mount["trip_at"] == pytest.approx(-0.5, abs=0.1)
            assert snapshot["physical_position"] == pytest.approx(-0.3, abs=0.1)
            # E132: machine-level HOMED requires all configured homing axes; this test homes X only.
            assert snapshot["cm.homed[0]"] == 1
            assert snapshot["cm.hold_state"] == 0, "homing left an unfinished feedhold"
            assert snapshot["mr.position[0]"] == pytest.approx(0, abs=0.005)
            assert mount["min_position"] > -2, "homing continued far beyond the switch"
            assert mount["max_position"] < 0.75, "unexpected positive travel during homing"
        completed = True
    finally:
        switches.unmount_all()
        if not completed:
            tg.reset()  # stop an unfinished homing cycle before restoring its settings
        switches.open("x", "min")
        switches.open("x", "max")
