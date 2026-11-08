"""A suspended host has lost the serial session, even if the board still answers."""
import pytest

from tinyg_hil import sender
from test_sender_offline import FakeTG, ok, board


class SuspendDetected(TimeoutError):
    pass


@pytest.mark.parametrize("when", ["write", "reply", "drain"])
def test_stream_rejects_suspend_instead_of_accepting_credits(monkeypatch, when):
    sessions = []

    class Power:
        def __init__(self):
            self.suspended = False
            self.started = self.closed = False
            sessions.append(self)

        def start(self):
            self.started = True

        def check(self):
            if self.suspended:
                raise SuspendDetected("host suspended for 503 seconds")

        def close(self):
            self.closed = True

    monkeypatch.setattr(sender, "AwakeSession", Power, raising=False)

    def answer(tr, data):
        if data.startswith(b'{"stat"'):
            board()(tr, data)
            if when == "drain" and sessions:
                sessions[-1].suspended = True
        else:
            tr.emit(ok(data))
            if when == "write" and sessions:
                sessions[-1].suspended = True

    tg = FakeTG(answer)
    if when == "reply":
        original = tg.transport.next_event

        def next_event(timeout):
            event = original(timeout)
            if event is not None and sessions:
                sessions[-1].suspended = True
            return event

        tg.transport.next_event = next_event
    with pytest.raises(SuspendDetected):
        sender.stream(tg, ["G1 X1 F600", "G1 X2"], "g2", timeout=2)
    assert len(sessions) == 1 and sessions[0].started and sessions[0].closed
    if when == "write":
        assert len(tg.transport.writes) == 1, "sent another line after host suspend"


@pytest.fixture
def clock_session(monkeypatch):
    from tinyg_hil import hostpower
    clock = {"awake": 10.0, "continuous": 910.0}
    monkeypatch.setattr(hostpower.sys, "platform", "test")
    monkeypatch.setattr(hostpower, "_continuous_clock", lambda: lambda: clock["continuous"])
    monkeypatch.setattr(hostpower.time, "monotonic", lambda: clock["awake"])
    session = hostpower.AwakeSession()
    session.start()
    return session, clock


def test_503_second_sleep_is_detected_and_stays_invalid(clock_session):
    from tinyg_hil.hostpower import HostSuspended
    session, clock = clock_session
    clock["awake"] += 0.47
    clock["continuous"] += 503.47
    for _ in range(2):
        with pytest.raises(HostSuspended, match="host suspended.*503.0"):
            session.check()
    session.close()


def test_cpu_delay_and_wall_clock_adjustments_are_not_sleep(clock_session, monkeypatch):
    from tinyg_hil import hostpower
    session, clock = clock_session
    clock["awake"] += 900
    clock["continuous"] += 900
    monkeypatch.setattr(hostpower.time, "time", lambda: -1000000)
    session.check()
    session.close()


def test_macos_assertion_has_pid_lease_and_is_released(monkeypatch):
    from tinyg_hil import hostpower
    calls = []

    class Child:
        stopped = False
        def poll(self): return 0 if self.stopped else None
        def terminate(self): self.stopped = True
        def wait(self, timeout): assert self.stopped and timeout == 1

    child = Child()
    monkeypatch.setattr(hostpower.sys, "platform", "darwin")
    monkeypatch.setattr(hostpower, "_continuous_clock", lambda: None)
    monkeypatch.setattr(hostpower.subprocess, "Popen", lambda args, **kwargs: calls.append(args) or child)
    session = hostpower.AwakeSession()
    session.start()
    assert calls == [["/usr/bin/caffeinate", "-is", "-w", str(hostpower.os.getpid())]]
    session.check()
    session.close()
    session.close()
    assert child.stopped


def test_exited_assertion_invalidates_the_session(clock_session):
    from tinyg_hil.hostpower import HostSuspended
    session, _ = clock_session
    class Child:
        def poll(self): return 1
        def wait(self, timeout): return 1
    session._child = Child()
    with pytest.raises(HostSuspended, match="sleep protection exited"):
        session.check()
    session.close()


def test_native_continuous_clock_adapter():
    from tinyg_hil import hostpower
    clock = hostpower._continuous_clock()
    if clock is None:
        pytest.skip("no continuous clock on this platform")
    before = clock()
    assert clock() >= before >= 0
