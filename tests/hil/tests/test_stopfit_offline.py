"""The stop estimator recovers known delays and jerks from synthetic firmware-profile step traces.

Traces come from the firmware's own tail model (segmented quintic, DDA tick grid, three grid phases)
with the fit started from a cruise guess 0.4% off. The bounds asserted here are the estimator errors
the E031 timing diagnostic adds to its gate (STOPFIT_DELAY_ERROR_S, STOPFIT_JERK_REL_ERROR).
"""
import math

import pytest

from tinyg_hil import stopfit


XJM_JERK = 20e6 / 216000.0      # xjm 20 (bench default), mm/s^3


def test_estimator_bounds_on_synthetic_firmware_traces():
    r = stopfit.validate(10.0, 1280.0, XJM_JERK)
    assert r["max_delay_error_s"] <= 0.0006, r["max_delay_error_s"]
    assert r["max_jerk_rel_error"] <= 0.01, r["max_jerk_rel_error"]
    assert all(c["rms_mm"] < 0.0005 for c in r["cases"])


def test_tail_covers_half_cruise_times_duration():
    # the symmetry the fit relies on: midpoint samples of the quintic sum to exactly n/2
    for T in (0.0213, 0.2191, 0.6573):
        segs = stopfit.tail_segments(10.0, T)
        assert math.isclose(sum(h * v for h, v in segs), 10.0 * T / 2, rel_tol=1e-9)
