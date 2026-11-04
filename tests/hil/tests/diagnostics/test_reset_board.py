"""Overnight helper: reset the board (^X) and confirm it answers, before a failed job is run again."""
import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.resets_board, pytest.mark.dialect("d449")]


def test_reset_board(tg):
    tg.reset()
    assert tg.stat() is not None
