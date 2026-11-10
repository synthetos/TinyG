"""gcodeplay.coverage(): status-report samples taken along a demo program's path, in time order, must
leave no gap - including programs that come back over their own path (mixed returns through its start)."""
import math

import pytest

from tinyg_hil import gcodeplay


def walk(pts, step=0.5, jitter=0.0015):
    """Samples every `step` mm along the polyline, alternately nudged off it like 3-decimal SR positions."""
    out, k = [], 0
    for a, b in zip(pts, pts[1:]):
        if a is None or b is None:
            continue
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
        for i in range(n):
            t = i / n
            k += 1
            j = jitter if k % 2 else -jitter
            out.append((round(a[0] + t * (b[0] - a[0]) + j, 3), round(a[1] + t * (b[1] - a[1]) - j, 3)))
    out.append(pts[-1])
    return out


@pytest.mark.parametrize("program", list(gcodeplay.PROGRAMS))
def test_samples_along_the_path_cover_it(program):
    path = gcodeplay.path(gcodeplay.PROGRAMS[program]())
    cover = gcodeplay.coverage(walk(path), path)
    assert cover["max_gap_mm"] <= 1.0, cover


def test_a_skipped_stretch_shows_as_a_gap():
    path = gcodeplay.path(gcodeplay.squares())
    samples = walk(path)
    cut = len(samples) // 3
    cover = gcodeplay.coverage(samples[:cut] + samples[cut + 60:], path)
    assert cover["max_gap_mm"] >= 25, cover


def test_a_setup_report_before_a_g92_program_starts_is_not_scored():
    """braid opens with G92 X0.327 Y-33.521: a positionless report sent before that line runs carries the
    pre-stream position (0, 0), which is off the program's path (a count-crlf braid run failed on it)."""
    path = gcodeplay.path(gcodeplay.LARGE_PROGRAMS["braid"]())
    status = [(0.048, {"posx": 0.0, "posy": 0.0, "stat": 4, "_raw": '{"sr":{"stat":4}}'})]
    status += [(1.0 + i, {"posx": x, "posy": y, "_raw": f'{{"sr":{{"posx":{x},"posy":{y}}}}}'})
               for i, (x, y) in enumerate(walk(path))]
    check = gcodeplay.path_check(status, path, 0.1, 0.01, 10)
    assert not check["failures"], check["failures"]
    assert check["samples"][0] != (0.0, 0.0) and len(check["samples"]) == len(status) - 1


def test_an_off_path_report_with_a_position_still_fails():
    path = gcodeplay.path(gcodeplay.LARGE_PROGRAMS["braid"]())
    status = [(1.0 + i, {"posx": x, "posy": y, "_raw": f'{{"sr":{{"posx":{x},"posy":{y}}}}}'})
              for i, (x, y) in enumerate(walk(path))]
    status.insert(5, (1.5, {"posx": 0.0, "posy": 0.0, "_raw": '{"sr":{"posx":0.000,"posy":0.000}}'}))
    check = gcodeplay.path_check(status, path, 0.1, 0.01, 10)
    assert any("off the commanded path" in f for f in check["failures"]), check["failures"]
