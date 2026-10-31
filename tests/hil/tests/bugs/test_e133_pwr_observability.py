"""E133: the harness cannot confirm motors are de-energized because pwr1-pwr4 are commented out.

config_app.c:173-184 wraps the pwr1..pwr6 rows in /* */ while the `pwr` group row (config_app.c:663)
and st_get_pwr (stepper.c:1246-1251) remain. Master has the rows. Same fix as E108: remove both
markers together.
"""
import pytest


def test_motor_power_observable(tg):
    tg.send_json({"md": None}, check=True)
    for m in range(1, 5):
        resp = tg.send_json({f"pwr{m}": None})
        assert resp.status == 0, resp.raw
        assert resp.body[f"pwr{m}"] == 0
