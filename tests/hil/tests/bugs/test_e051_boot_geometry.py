"""Regression: E051's runtime guard must not reject the startup EEPROM load.

This passes on stock and must pass on integration builds too; no expected-failure marker.
Only setting writes/reset/readback are used, with no commanded motion.
"""
import pytest

pytestmark = [pytest.mark.dialect("d449"), pytest.mark.persists, pytest.mark.resets_board]


def test_motor_geometry_survives_reset(tg, keep):
    tokens = [f"{m}{suffix}" for m in range(1, 5) for suffix in ("sa", "tr", "mi")]
    keep(*tokens)
    tg.wait_idle()
    expected = {f"{m}{suffix}": value for m in range(1, 5)
                for suffix, value in (("sa", 0.9), ("tr", 5.0), ("mi", 8))}
    for token, value in expected.items():
        tg.set(token, value, allow="geometry persistence regression; no motion")
        assert tg.get(token) == pytest.approx(value)
    tg.reset()
    actual = {token: tg.get(token) for token in tokens}
    assert actual == pytest.approx(expected), actual
