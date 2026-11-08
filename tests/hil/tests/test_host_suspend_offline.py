"""A suspended host has lost the serial session, even if the board still answers."""
import pytest

from tinyg_hil import sender
from test_sender_offline import FakeTG, ok, board


class SuspendDetected(TimeoutError):
    pass


@pytest.mark.xfail(strict=True, reason="N97: stream ignores host suspend and can trust lost serial credits")
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
