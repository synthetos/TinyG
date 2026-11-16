"""Offline: the soak cap's feedhold confirmation (tinyg_hil.soak.confirm_hold) accepts only a hold reported after
the request that then stays held for the settle window; stale reports and silence never pass."""
import pytest

from tinyg_hil.soak import CapStopTimeout, confirm_hold, fold_status

HOLD, SETTLE = 6, 1.0


class Ev:
    def __init__(self, t, sr):
        self.t, self.kind, self.data = t, "status", {"sr": sr}


class Feed:
    """Fake clock + event queue: next_event(dt) advances time and returns the first event due by then."""
    def __init__(self, events, t0=100.0):
        self.now, self.q = t0, sorted(events, key=lambda e: e.t)

    def clock(self):
        return self.now

    def next_event(self, dt):
        if self.q and self.q[0].t <= self.now + dt:
            ev = self.q.pop(0)
            self.now = max(self.now, ev.t)
            return ev
        self.now += dt
        return None


def run(history, events, timeout=6.0):
    feed = Feed(events)
    state = confirm_hold(feed.next_event, fold_status(history, 100), 100, HOLD, feed.clock, SETTLE, timeout)
    return state, feed.now


def test_fresh_hold_that_stays_held_passes_after_the_settle():
    _, now = run([Ev(95, {"stat": 5, "vel": 300})], [Ev(100.6, {"stat": 6, "vel": 0})])
    assert now >= 101.6


def test_old_hold_then_resume_then_new_hold_needs_the_new_hold():
    history = [Ev(90, {"stat": 6, "vel": 0}), Ev(95, {"stat": 5, "vel": 300})]
    _, now = run(history, [Ev(100.4, {"stat": 5, "vel": 250}), Ev(101.0, {"stat": 6, "vel": 0})])
    assert now >= 102.0


def test_delayed_old_hold_report_crossing_the_request_is_overtaken_by_its_resume():
    # an old hold report and its resume, both sent before '!', received after it; then the cap's own hold
    events = [Ev(100.05, {"stat": 6, "vel": 0}), Ev(100.30, {"stat": 5, "vel": 280}), Ev(101.2, {"stat": 6, "vel": 0})]
    _, now = run([Ev(95, {"stat": 5, "vel": 300})], events)
    assert now >= 102.2          # the first (stale) hold was cancelled inside the window


def test_pre_request_events_still_queued_are_ignored():
    history = [Ev(98, {"stat": 6, "vel": 0}), Ev(99, {"stat": 5, "vel": 300})]
    events = [Ev(98, {"stat": 6, "vel": 0}), Ev(99, {"stat": 5, "vel": 300}), Ev(100.5, {"stat": 6, "vel": 0})]
    _, now = run(history, events)
    assert now >= 101.5


def test_historical_hold_with_no_fresh_report_times_out():
    with pytest.raises(CapStopTimeout):
        run([Ev(90, {"stat": 6, "vel": 0})], [])


def test_moving_machine_with_no_reports_times_out():
    with pytest.raises(CapStopTimeout):
        run([Ev(99, {"stat": 5, "vel": 300})], [])


def test_hold_that_does_not_stay_held_keeps_waiting():
    with pytest.raises(CapStopTimeout):
        run([Ev(95, {"stat": 5, "vel": 300})],
            [Ev(100.5, {"stat": 6, "vel": 0}), Ev(100.9, {"stat": 5, "vel": 100})], timeout=4)
