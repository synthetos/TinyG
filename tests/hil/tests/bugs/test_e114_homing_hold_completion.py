"""E114 integration coverage: repeated homing through mounted software switches.

Requires TEST_HOOKS=1, --ice and --allow-motion on the bare board. This test
checks complete search/latch/backoff cycles, but does not force the narrow
PENDING-to-HOLD interleaving: a stock pass is not evidence against E114.
The deterministic ordering regression is tests/host/test_homing_hold_completion.py.
"""
import time

import pytest

pytestmark = [pytest.mark.ice, pytest.mark.motion, pytest.mark.resets_board,
              pytest.mark.dialect("d449"), pytest.mark.profile("switches-no")]


def test_repeated_homing_completes_after_switch_holds(tg, keep, switches, ice, record):
    if "test_hook" not in ice.symbols.sym:
        pytest.skip("requires TEST_HOOKS=1; halted pin writes do not exercise port edges")
    settings = {"xsn": 1, "xsx": 2, "xtn": -2, "xtm": 2,
                "xsv": 120, "xlv": 60, "xlb": 1, "xzb": 0.2}
    # Keep an unused opposite limit configured: disabling it exposes separate E055.
    keep(*settings)
    for token, value in settings.items():
        tg.set(token, value, allow="small X homing envelope for software-switch test")
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
            switches.trips.clear()
            switches.samples.clear()
            switches.mount("x", "min", at=-0.5)
            tg.send_gcode("G28.2 X0", check=True, allow="home X against software switch at -0.5 mm")
            deadline = time.monotonic() + 25
            snapshots = []
            while time.monotonic() < deadline:
                assert switches.error is None, f"switch follower failed: {switches.error!r}"
                snapshot = ice.peek_many(["cm.cycle_state", "cm.homing_state", "cm.homed[0]",
                                          "cm.hold_state", "mr.position[0]"])
                snapshots.append(snapshot)
                if snapshot["cm.cycle_state"] == 0 and switches.trips:
                    break
                time.sleep(0.05)
            else:
                record("homing_timeout_snapshots", snapshots[-30:])
                pytest.fail("homing did not finish before the deadline")
            switches.unmount_all()
            assert switches.error is None, f"switch follower failed: {switches.error!r}"
            trips = [entry for entry in switches.trips if entry[1:3] == ("x", "min")]
            active = [entry for entry in trips if entry[3]]
            inactive = [entry for entry in trips if not entry[3]]
            positions = [entry[2] for entry in switches.samples]
            result = {"iteration": iteration, "trips": trips, "final": snapshot,
                      "min_position": min(positions) if positions else None,
                      "max_position": max(positions) if positions else None}
            rounds.append(result)
            record("homing_rounds", rounds)
            assert active and inactive, "homing did not close and then release the mounted switch"
            assert active[0][0] < inactive[-1][0], "no switch release after the search closure"
            assert snapshot["cm.homing_state"] == 1 and snapshot["cm.homed[0]"] == 1
            assert snapshot["cm.hold_state"] == 0, "homing left an unfinished feedhold"
            assert snapshot["mr.position[0]"] == pytest.approx(0, abs=0.005)
            assert positions and min(positions) > -2, "homing continued far beyond the switch"
            assert max(positions) < 0.75, "unexpected positive travel during homing"
        completed = True
    finally:
        switches.unmount_all()
        if not completed:
            tg.reset()  # stop an unfinished homing cycle before restoring its settings
        switches.open("x", "min")
        switches.open("x", "max")
