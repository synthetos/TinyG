"""Offline: the soak cap's feedhold confirmation (tinyg_hil.soak.confirm_hold) never trusts a report from before the
request, and only accepts an already-stopped machine after a silence that rules motion out."""
import pytest

from tinyg_hil.soak import CapStopTimeout, confirm_hold, fold_status

HOLD, STOPPED = 6, {6, 3, 1, 4}


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


def test_old_hold_then_resume_then_new_hold_needs_the_fresh_report():
    history = [Ev(90, {"stat": 6, "vel": 0}), Ev(95, {"stat": 5, "vel": 300})]
    # the request at 100; delayed reports: still running at 100.4, held at 101.0
    feed = Feed([Ev(100.4, {"stat": 5, "vel": 250}), Ev(101.0, {"stat": 6, "vel": 0})])
    how, st = confirm_hold(feed.next_event, fold_status(history, 100), 100, HOLD, STOPPED, feed.clock)
    assert how == "fresh_hold" and feed.now >= 101.0


def test_historical_hold_is_not_proof_when_a_resume_report_is_still_queued():
    # history says held, but a report from after the request shows motion again (holder's final '~')
    history = [Ev(99, {"stat": 6, "vel": 0})]
    feed = Feed([Ev(100.2, {"stat": 5, "vel": 120}), Ev(100.9, {"stat": 6, "vel": 0})])
    how, _ = confirm_hold(feed.next_event, fold_status(history, 100), 100, HOLD, STOPPED, feed.clock)
    assert how == "fresh_hold" and feed.now >= 100.9


def test_pre_request_events_still_queued_are_ignored():
    history = [Ev(98, {"stat": 6, "vel": 0}), Ev(99, {"stat": 5, "vel": 300})]
    feed = Feed([Ev(98, {"stat": 6, "vel": 0}), Ev(99, {"stat": 5, "vel": 300}),   # already folded, still queued
                 Ev(100.5, {"stat": 6, "vel": 0})])
    how, _ = confirm_hold(feed.next_event, fold_status(history, 100), 100, HOLD, STOPPED, feed.clock)
    assert how == "fresh_hold" and feed.now >= 100.5


def test_already_held_machine_is_accepted_only_after_the_silence():
    history = [Ev(90, {"stat": 6, "vel": 0})]
    feed = Feed([])
    how, _ = confirm_hold(feed.next_event, fold_status(history, 100), 100, HOLD, STOPPED, feed.clock, silence=2.0)
    assert how == "already_stopped" and feed.now >= 102.0


def test_moving_machine_with_no_reports_is_never_accepted():
    history = [Ev(99, {"stat": 5, "vel": 300})]
    feed = Feed([])
    with pytest.raises(CapStopTimeout):
        confirm_hold(feed.next_event, fold_status(history, 100), 100, HOLD, STOPPED, feed.clock, timeout=5)


def test_fresh_report_cancels_the_already_stopped_shortcut():
    # seeded as held, a fresh report shows motion: only a later fresh hold may pass
    history = [Ev(90, {"stat": 6, "vel": 0})]
    feed = Feed([Ev(100.1, {"stat": 5, "vel": 50})])
    with pytest.raises(CapStopTimeout):
        confirm_hold(feed.next_event, fold_status(history, 100), 100, HOLD, STOPPED, feed.clock, timeout=5)
