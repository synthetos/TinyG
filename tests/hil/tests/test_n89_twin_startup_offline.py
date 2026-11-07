"""Accept only a leading one-pulse phase difference, never a missing pulse later in a window."""
import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location("twin_diagnostic", Path(__file__).parent / "diagnostics/test_bench3_file_mso.py")
windows = importlib.util.module_from_spec(spec)
spec.loader.exec_module(windows)


def test_one_leading_pulse_with_all_successors_aligned():
    a = [1.14466258, 1.14510325, 1.14556399, 1.14602466]
    b = [1.14510585, 1.14556659, 1.14602726]
    assert windows._one_startup_pulse(a, b)
    assert windows._one_startup_pulse(b, a)


@pytest.mark.parametrize("a,b", [
    ([1, 2, 3, 4], [1, 3, 4]),  # a missing pulse in the middle
    ([1, 2, 3, 4], [1, 2, 3]),  # a missing final pulse
    ([1, 2, 3, 4], [3, 4]),     # two excess pulses
    ([1, 2, 3], [1, 2, 3]),     # not a startup exception
    ([1, 2, 3], [2.001, 3.001]),  # remaining trains are out of phase
])
def test_does_not_excuse_other_pulse_differences(a, b):
    assert not windows._one_startup_pulse(a, b)
