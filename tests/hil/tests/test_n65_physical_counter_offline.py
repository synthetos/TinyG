"""Mount frames and final physical positions include the unretired final segment."""
from contextlib import nullcontext
from types import SimpleNamespace

import pytest

from tinyg_hil.switches import SoftSwitches


class Ice:
    HOOK_DRIVE_LOW, HOOK_RELEASE = 1, 0
    board = SimpleNamespace(switches=["x"], symbols={"runtime_position": "mr.position"}, axis_index=lambda _: 0)

    def __init__(self):
        self.writes = {}

    def peek_many(self, names):
        values = {"test_hook.steps[0]": 100, "en.en[0].steps_run": 6, "mr.position[0]": 0}
        return {name: values[name] for name in names}

    def poke(self, name, value):
        self.writes[name] = value

    def halted(self):
        return nullcontext()


def test_mount_origin_counts_pending_steps(monkeypatch):
    monkeypatch.setattr("tinyg_hil.switches.time.sleep", lambda _: None)
    ice = Ice()
    switches = SoftSwitches(ice, geometry=lambda _: (0, 1280))
    switches._mount_in_firmware("x", "min", -0.5)
    assert ice.writes["test_hook.mount[0].at"] == 106 - 640
    assert switches.frames[("x", "min")][0] == 106


def test_physical_position_counts_pending_steps():
    switches = SoftSwitches(Ice())
    switches.frames[("x", "min")] = (100, 0, 1280, 0)
    assert switches.physical_position("x") == pytest.approx(6 / 1280)
