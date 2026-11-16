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


class Backlog(Feed):
    """Events already queued while the host was busy: the clock is already past them when reading starts."""
    def __init__(self, events, start):
        super().__init__(events)
        self.now = start

    def next_event(self, dt):
        if self.q:
            return self.q.pop(0)                   # already received; reading does not advance the clock
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


def test_backlog_with_the_stale_holds_resume_unread_is_not_accepted():
    # host busy until 102: a stale hold (100.05) and its resume (100.30) are both queued, then the cap's hold (102.5)
    feed = Backlog([Ev(100.05, {"stat": 6, "vel": 0}), Ev(100.30, {"stat": 5, "vel": 280})], start=102.0)
    feed.q.append(Ev(102.5, {"stat": 6, "vel": 0}))
    with pytest.raises(CapStopTimeout):          # the queued fresh hold at 102.5 is read at 102.0: then it must settle
        confirm_hold(feed.next_event, fold_status([Ev(95, {"stat": 5, "vel": 300})], 100), 100, HOLD,
                     feed.clock, SETTLE, timeout=2.5)
    feed2 = Backlog([Ev(100.05, {"stat": 6, "vel": 0}), Ev(100.30, {"stat": 5, "vel": 280}),
                     Ev(102.5, {"stat": 6, "vel": 0})], start=102.0)
    state = confirm_hold(feed2.next_event, fold_status([Ev(95, {"stat": 5, "vel": 300})], 100), 100, HOLD,
                         feed2.clock, SETTLE, timeout=6.0)
    assert state["stat"] == 6 and feed2.now >= 103.0   # settle counted from consumption, after the resume was read


def test_backlog_ending_in_running_never_accepts_the_earlier_hold():
    feed = Backlog([Ev(100.05, {"stat": 6, "vel": 0}), Ev(100.30, {"stat": 5, "vel": 280})], start=102.0)
    with pytest.raises(CapStopTimeout):
        confirm_hold(feed.next_event, fold_status([Ev(95, {"stat": 5, "vel": 300})], 100), 100, HOLD,
                     feed.clock, SETTLE, timeout=4.0)


class Endless(Feed):
    """A stream that never goes quiet: a RUN report every 0.1 s, the clock advancing with each one."""
    def next_event(self, dt):
        self.now += 0.1
        return Ev(self.now, {"stat": 5, "vel": 200})


def test_a_never_quiet_stream_still_hits_the_deadline():
    feed = Endless([])
    with pytest.raises(CapStopTimeout):
        confirm_hold(feed.next_event, {}, 100, HOLD, feed.clock, SETTLE, timeout=3.0)
    assert feed.now <= 103.2


def test_a_held_candidate_after_the_deadline_is_not_accepted():
    feed = Feed([Ev(102.9, {"stat": 6, "vel": 0})])
    with pytest.raises(CapStopTimeout):
        confirm_hold(feed.next_event, fold_status([Ev(95, {"stat": 5, "vel": 300})], 100), 100, HOLD,
                     feed.clock, SETTLE, timeout=3.0)
