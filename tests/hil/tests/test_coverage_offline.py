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
