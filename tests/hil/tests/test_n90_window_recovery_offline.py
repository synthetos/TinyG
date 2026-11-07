"""A refused arc must not leave the following MSO window in its alarm."""
import importlib.util
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location(
    "window_diagnostic", Path(__file__).parent / "diagnostics/test_bench3_file_mso.py")
diagnostic = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(diagnostic)


class Dialect:
    idle_stats = (1, 3, 4)

    def stat(self, name):
        return {"ALARM": 2}[name]

    def stats(self, *names):
        return tuple({"SHUTDOWN": 10, "PANIC": 13}[n] for n in names)


class Board:
    dialect = Dialect()

    def __init__(self, state):
        self.state, self.calls = state, []

    def clear_alarm(self):
        self.calls.append("clear")
        self.state = 3

    def wait_idle(self, timeout):
        self.calls.append("wait")
        assert self.state in self.dialect.idle_stats

    def stat(self):
        return self.state

    def get(self, token):
        assert token == "qr"
        return 32


def test_refused_arc_recovers_before_next_window():
    tg = Board(2)  # preceding window's G2 returned 154
    assert diagnostic._recover_refused_window(tg, 2)
    assert tg.calls == ["clear", "wait"]
    assert tg.stat() == 3  # the following preamble is accepted rather than refused with 204


@pytest.mark.parametrize("state", [10, 13])
def test_terminal_failure_does_not_continue_into_another_window(state):
    tg = Board(state)
    assert not diagnostic._recover_refused_window(tg, state)
    assert not tg.calls


def test_non_alarm_refusal_does_not_reset_the_board():
    tg = Board(3)
    assert diagnostic._recover_refused_window(tg, 3)
    assert not tg.calls
