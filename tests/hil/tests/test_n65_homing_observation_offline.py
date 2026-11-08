"""The mounted-switch integration test must not halt during homing."""
from contextlib import contextmanager
import importlib.util
from pathlib import Path
from types import SimpleNamespace


def test_homing_runs_without_debugger_samples(monkeypatch):
    spec = importlib.util.spec_from_file_location("homing_test", Path(__file__).parent / "bugs/test_e114_homing_hold_completion.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module.time, "sleep", lambda _: None)
    observations = {}

    class Board:
        moving = False
        waits = 0

        def get(self, token):
            return 0 if token.endswith("ma") and token.startswith("1") else 1

        def set(self, *args, **kwargs):
            pass

        def send_gcode(self, line, **kwargs):
            if line.startswith("G28.2"):
                self.moving = True

        def wait_idle(self, **kwargs):
            self.waits += 1
            self.moving = False

        def reset(self):
            self.moving = False

    tg = Board()

    class Switches:
        frames = {}
        error = None

        def unmount_all(self):
            self.frames.clear()

        def open(self, *args):
            pass

        def mount(self, *args, **kwargs):
            assert not tg.moving
            self.frames[("x", "min")] = 1

        def mount_status(self, *args):
            assert not tg.moving
            return {"closed": False, "trips": 1, "trip_at": -0.5,
                    "min_position": -0.725, "max_position": 0.0}

        def physical_position(self, axis):
            assert not tg.moving
            return -0.3

    class Ice:
        symbols = SimpleNamespace(sym={"test_hook": 1}, resolve=lambda _: 0)

        @contextmanager
        def halted(self):
            assert not tg.moving, "ICE halt overlapped the switch stimulus"
            yield self

        def peek_many(self, names):
            assert not tg.moving
            return {name: (1 if name == "cm.homed[0]" else 0) for name in names}

    module.test_repeated_homing_completes_after_switch_holds(
        tg, lambda *args: None, Switches(), Ice(), observations.__setitem__)
    assert tg.waits >= 6  # origin command and homing completion, for each round
    assert len(observations["homing_rounds"]) == 3
