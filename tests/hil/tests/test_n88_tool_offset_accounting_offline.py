"""Tool offsets are checked against displacement, including an endpoint correction pair."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

spec = importlib.util.spec_from_file_location("tlo_diagnostic", Path(__file__).parent / "diagnostics/test_tool_offset_mso.py")
tlo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tlo)


@pytest.mark.parametrize("polarity", [0, 1])
def test_reverse_and_forward_correction_counts_as_zero_travel(monkeypatch, tmp_path, polarity):
    # 8962 pulses, including one reverse pulse: exactly 8960 signed steps.
    step = [(i + phase, level) for i in range(8962) for phase, level in ((0.1, 1), (0.2, 0))]
    direction = [(8960.0, 1 - polarity), (8961.0, polarity)]
    edges = {"M1 step": step, "M1 dir": direction}

    class Capture:
        def __init__(self, cfg, digital, **kwargs):
            assert set(digital) == {"M1 step", "M1 dir"}

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def edges(self):
            return edges

        def initial(self):
            return {"M1 step": 0, "M1 dir": polarity}

    class Board:
        guard = SimpleNamespace(check_gcode=lambda *a, **kw: None)

        def get(self, token):
            return polarity if token == "1po" else 0

        def send_gcode(self, *args, **kwargs):
            pass

        def wait_idle(self, **kwargs):
            pass

    monkeypatch.setattr(tlo.mso, "Capture", Capture)
    monkeypatch.setattr(tlo.time, "sleep", lambda _: None)
    request = SimpleNamespace(config=SimpleNamespace(_tinyg_recorder=SimpleNamespace(dir=tmp_path)))
    counts, _ = tlo._steps(Board(), {}, request, "correction-pair", ["G0 Z0"], channels=("M1 step",))
    assert counts["M1 step"] == 8962
    assert counts["M1 net"] == 8960
