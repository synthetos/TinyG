"""Offline checks for tinyg_hil.override_steps: the override step tests must fail on wrong-direction travel and on a
feedhold that never completes."""
import pytest

from tinyg_hil.override_steps import pick_axis, whole_step_distance
from tinyg_hil.override_steps import (HOLD_COMPLETE, HoldNotReached, dir_level, signed_steps, wait_for_hold,
                                      within)


def train(n, t0=0.0, dt=1e-4):
    return [t0 + k * dt for k in range(n)]


def test_dir_level_follows_polarity():
    assert dir_level(True, 0) == 0 and dir_level(False, 0) == 1
    assert dir_level(True, 1) == 1 and dir_level(False, 1) == 0


def test_forward_only():
    net, back, fwd = signed_steps(train(1000), [], 0, forward_level=0)
    assert (net, back, len(fwd)) == (1000, 0, 1000)


def test_forward_plus_one_reverse_tail():
    rises = train(1001) + [0.2]                          # 1001 forward, then DIR flips and one pulse back
    net, back, fwd = signed_steps(rises, [(0.15, 1)], 0, forward_level=0)
    assert (net, back, len(fwd)) == (1000, 1, 1001)
    assert within(net, 1000)


def test_wholly_reversed_travel_fails_the_target():
    net, back, fwd = signed_steps(train(1000), [], 1, forward_level=0)      # every pulse at the reverse level
    assert net == -1000 and back == 1000 and fwd == []
    assert not within(net, 1000)


def test_missing_initial_dir_is_an_error():
    with pytest.raises(ValueError):
        signed_steps(train(10), [], None, forward_level=0)


def test_leg_direction_matters():
    assert within(102401, 102400) and within(-102399, -102400)
    assert not within(-102400, 102400)                    # an outbound move in the wrong direction
    assert not within(102400, -102400)                    # a return move in the wrong direction


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, s):
        self.t += s


def test_hold_reached():
    c = Clock()
    states = iter([4, 5, 5, HOLD_COMPLETE])
    assert wait_for_hold(lambda: next(states), timeout=5, poll=0.1, clock=c, sleep=c.sleep) == pytest.approx(0.3)


def test_missed_hold_cannot_count_as_coverage():
    c = Clock()
    with pytest.raises(HoldNotReached):
        wait_for_hold(lambda: 0, timeout=1.0, poll=0.1, clock=c, sleep=c.sleep)     # the ! was never acted on


def test_unfinished_hold_cannot_count_as_coverage():
    c = Clock()
    with pytest.raises(HoldNotReached):
        wait_for_hold(lambda: 4, timeout=1.0, poll=0.1, clock=c, sleep=c.sleep)     # still decelerating


def test_whole_step_distance_single_motor():
    assert whole_step_distance([1280.0], 80.0) == (80.0, [102400])


def test_whole_step_distance_mixed_gearing():
    # bench 1 X: motor 1 at 1280 steps/mm, motor 4 at 360*8/(1.8*360) = 4.444... steps/mm
    dist, targets = whole_step_distance([1280.0, 360 * 8 / (1.8 * 360)], 80.0)
    assert abs(dist - 80.0) <= 3.0 and targets == [round(dist * 1280), round(dist * 40 / 9)]
    assert (dist, targets) == (80.1, [102528, 356])      # exact on both motors


def test_pick_axis_prefers_finest_then_falls_back():
    assert pick_axis([("x", [1280.0]), ("y", [640.0])], 80.0)[0] == "x"
    assert pick_axis([("x", [1280.0, 1280.0 * 1.0000001]), ("y", [640.0])], 80.0)[0] == "y"   # no common whole step on X


def test_pick_axis_none():
    assert pick_axis([("x", [1280.0, 1280.0 * 1.0000001])], 80.0) is None
