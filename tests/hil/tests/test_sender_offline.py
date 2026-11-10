"""The streaming sender fails or reports failure for every way a stream can go wrong, against a fake board.

The fake board answers each line it receives with a scripted response; the stat poll after the last
answer gets a scripted machine state. Covered: the wait for the machine to stop ends idle, alarmed,
shut down, panicked or timed out, and only idle is a pass; exception reports during that wait count;
a response with no line outstanding or without a valid footer fails the stream; character counting
never exceeds its byte budget and refuses a line that cannot fit; an RTS stream whose output is held
back keeps reading and stalls cleanly instead of blocking in a write. The path verdict fails with no
status reports and with a gap in coverage.
"""
import json
import queue
import time

import pytest

from tinyg_hil import gcodeplay, sender
from tinyg_hil.protocol import parse_line


@pytest.fixture(autouse=True)
def no_real_power_assertion(monkeypatch):
    # Fake serial boards: assertion/clock behavior has separate coverage.
    class Power:
        def start(self): pass
        def check(self): pass
        def close(self): pass
    monkeypatch.setattr(sender, "AwakeSession", Power)


class FakeSerial:
    def __init__(self):
        self.write_timeout = 5
        self.rtscts = False
        self.out_waiting = 0


class FakeTransport:
    def __init__(self, board):
        self.ser = FakeSerial()
        self.flow = []
        self.events = queue.Queue()
        self.writes = []
        self.board = board

    def emit(self, raw):
        self.events.put(parse_line(raw, time.monotonic()))

    def write(self, data, drain_timeout=2.0):
        self.writes.append((data, self.ser.write_timeout))
        self.board(self, data)

    def next_event(self, timeout):
        try:
            return self.events.get(timeout=max(timeout, 0))
        except queue.Empty:
            return None


class FakeTG:
    def __init__(self, board):
        self.transport = FakeTransport(board)

    def get(self, token):
        return 30 if token == "rx" else 0.0


def ok(raw_line, status=0, count=None):
    return json.dumps({"r": {}, "f": [3, status, len(raw_line) if count is None else count]})


def board(stat=3, extra=None, footer=True, exception_on_stat=False):
    """Answers each G-code line; answers the stat poll with `stat` (None: never answers it)."""
    def answer(tr, data):
        text = data.decode()
        if text.startswith('{"stat"'):
            if exception_on_stat:
                tr.emit('{"er":{"fb":450.01,"st":27,"msg":"Move less than minimum length"}}')
            if stat is not None:
                tr.emit(json.dumps({"r": {"stat": stat}, "f": [3, 0, 8]}))
            return
        for line in text.splitlines():
            if line.strip():
                tr.emit(ok(line) if footer else json.dumps({"r": {}}))
                if extra:
                    tr.emit(extra)
    return answer


LINES = ["G1 X1 F600", "G1 X2", "G1 X3"]


@pytest.mark.parametrize("stat,completion", [(3, "idle"), (1, "idle"), (4, "idle"), (2, "alarm"),
                                             (12, "shutdown"), (13, "panic")])
def test_completion_state_is_explicit(stat, completion):
    res = sender.stream(FakeTG(board(stat=stat)), LINES, "count", timeout=5)
    assert res.acked == res.lines == 3
    assert res.completion == completion
    assert res.ok == (completion == "idle")


def test_completion_times_out_when_the_machine_never_stops():
    res = sender.stream(FakeTG(board(stat=None)), ["G1 X1 F600"], "count", timeout=1.0)
    assert res.acked == res.lines == 1 and not res.errors and not res.exceptions
    assert res.completion == "timeout" and not res.ok


def test_exception_during_completion_counts():
    res = sender.stream(FakeTG(board(exception_on_stat=True)), LINES, "count", timeout=5)
    assert res.completion == "idle"
    assert res.exceptions and not res.ok


def test_response_with_no_line_outstanding_fails():
    with pytest.raises(sender.StreamProtocolError, match="no line outstanding") as e:
        sender.stream(FakeTG(board(extra=json.dumps({"r": {}, "f": [3, 0, 5]}))), ["G1 X1 F600"], "line", timeout=5)
    assert e.value.result.acked == 1          # the stray second response is not taken as credit


def test_response_without_footer_fails():
    with pytest.raises(sender.StreamProtocolError, match="no valid footer"):
        sender.stream(FakeTG(board(footer=False)), LINES, "g2", timeout=5)


def test_count_mode_refuses_a_line_that_cannot_fit():
    tg = FakeTG(board())
    long_line = "G1 X1 (" + "c" * sender.RX_COUNT_LIMIT + ")"
    for mode in ("count", "count-crlf"):
        with pytest.raises(sender.StreamLineTooLong):
            sender.stream(tg, ["G1 X0", long_line], mode, timeout=5)
    assert tg.transport.writes == []


@pytest.mark.parametrize("wire_size", [193, 225, 250])
def test_rts_refuses_oversized_line_before_starting_job(wire_size):
    # N04/N102: one oversized line can lower CTS during a hold and trap resume.
    # Count the LF too, and reject the whole job before its first move is sent.
    line = "G1 X1 (" + "p" * (wire_size - 9) + ")"
    assert len(line.encode()) + 1 == wire_size
    tg = FakeTG(board())
    with pytest.raises(sender.StreamLineTooLong):
        sender.stream(tg, ["G1 X0", line], "rts", timeout=1)
    assert tg.transport.writes == []


def test_rts_accepts_a_line_at_the_byte_limit():
    line = "G1 X1 (" + "p" * (sender.RTS_INFLIGHT_LIMIT - 9) + ")"
    tg = FakeTG(board())
    assert sender.stream(tg, [line], "rts", timeout=1).ok
    assert len(tg.transport.writes[0][0]) == sender.RTS_INFLIGHT_LIMIT


def test_count_mode_keeps_its_byte_budget():
    """The board answers one line only when the host is waiting for input, so the host fills its budget;
    the bytes the board holds unanswered never exceed RX_COUNT_LIMIT."""
    held, peak = [], {"bytes": 0}

    def lazy(tr, data):
        if data.startswith(b'{"stat"'):
            tr.emit(json.dumps({"r": {"stat": 3}, "f": [3, 0, 8]}))
            return
        held.append(data)
        peak["bytes"] = max(peak["bytes"], sum(len(d) for d in held))
    tg = FakeTG(lazy)
    real_next = tg.transport.next_event

    def next_event(timeout):
        ev = real_next(0)
        if ev is None and held:                 # nothing waiting: the board finishes its oldest line
            tg.transport.emit(ok(held.pop(0).decode().rstrip("\n")))
            ev = real_next(0)
        return ev
    tg.transport.next_event = next_event
    lines = [f"G1 X{k} Y{k} (pad {'p' * (k % 40)})" for k in range(60)]
    res = sender.stream(tg, lines, "count", timeout=10)
    assert res.ok, (res.completion, res.errors)
    assert sender.RX_COUNT_LIMIT - 60 < peak["bytes"] <= sender.RX_COUNT_LIMIT, peak


def test_rts_held_output_stalls_without_blocking(monkeypatch):
    """CTS held: the driver queue stays full, so no line is written into it; the loop keeps reading and the
    stall watchdog fails the stream. Writes run with a bounded timeout, restored afterwards."""
    monkeypatch.setattr(sender, "STALL", 0.5)
    tg = FakeTG(board())
    tg.transport.ser.out_waiting = sender.TTY_ROOM          # full and not draining
    t0 = time.monotonic()
    with pytest.raises(sender.StreamStalled):
        sender.stream(tg, LINES, "rts", timeout=10)
    assert time.monotonic() - t0 < 3
    assert tg.transport.writes == []
    assert tg.transport.ser.write_timeout == 5 and tg.transport.ser.rtscts is False


def test_rts_writes_are_bounded_while_flowing():
    tg = FakeTG(board())
    res = sender.stream(tg, LINES, "rts", timeout=5)
    assert res.ok
    timeouts = {t for d, t in tg.transport.writes if not d.startswith(b'{"stat"')}
    assert timeouts == {sender.TTY_WRITE_TIMEOUT}
    assert tg.transport.ser.write_timeout == 5



def test_rts_bounds_unanswered_bytes_when_the_driver_hides_its_queue():
    """N102: the macOS FTDI driver accepts kilobytes while out_waiting reads near zero, so a '!' written
    mid-stream reached the board ~90 s late, and a hold with more unanswered bytes than the RTS marks allow locks the link. The board here answers one line only when the host waits for
    input and out_waiting always reads 0; the bytes it holds unanswered must stay within RTS_INFLIGHT_LIMIT."""
    held, peak = [], {"bytes": 0}

    def lazy(tr, data):
        if data.startswith(b'{"stat"'):
            tr.emit(json.dumps({"r": {"stat": 3}, "f": [3, 0, 8]}))
            return
        held.append(data)
        peak["bytes"] = max(peak["bytes"], sum(len(d) for d in held))
    tg = FakeTG(lazy)
    real_next = tg.transport.next_event

    def next_event(timeout):
        ev = real_next(0)
        if ev is None and held:
            tg.transport.emit(ok(held.pop(0).decode().rstrip("\n")))
            ev = real_next(0)
        return ev
    calls = {"n": 0}

    def slow_next(timeout):                     # a planner-bound board: finishes a line every 8th poll
        calls["n"] += 1
        return next_event(timeout) if calls["n"] % 8 == 0 else real_next(0)
    tg.transport.next_event = slow_next
    lines = [f"G1 X{k} Y{k} (pad {'p' * (k % 40)})" for k in range(120)]
    assert sum(len(x) + 1 for x in lines) > 4 * sender.RTS_INFLIGHT_LIMIT
    res = sender.stream(tg, lines, "rts", timeout=10)
    assert res.ok, (res.completion, res.errors)
    assert sender.RTS_INFLIGHT_LIMIT - 60 < peak["bytes"] <= sender.RTS_INFLIGHT_LIMIT, peak

PATH = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]


def _status(points, vel=600):
    return [(k * 0.1, {"posx": x, "posy": y, "vel": vel}) for k, (x, y) in enumerate(points)]


def test_path_check_fails_without_status_reports():
    check = gcodeplay.path_check([], PATH, 0.1, 0.05, 10)
    assert check["worst"] is None
    assert any("no evidence" in f for f in check["failures"])
    assert any("no status report" in f for f in check["failures"])


def test_path_check_fails_on_a_gap_and_passes_on_full_coverage():
    full = [(x * 0.5, 0.0) for x in range(21)] + [(10.0, y * 0.5) for y in range(1, 21)]
    assert gcodeplay.path_check(_status(full), PATH, 0.1, 0.05, 10)["failures"] == []
    gappy = [p for p in full if not (2.0 < p[0] < 9.0 and p[1] == 0.0)]
    failures = gcodeplay.path_check(_status(gappy), PATH, 0.1, 0.05, 10)["failures"]
    assert len(failures) == 1 and "no status report" in failures[0]


def test_path_check_fails_off_path():
    pts = [(x * 0.5, 0.0) for x in range(21)] + [(10.0, y * 0.5) for y in range(1, 21)] + [(5.0, 0.2)]
    failures = gcodeplay.path_check(_status(pts), PATH, 0.1, 0.05, 10)["failures"]
    assert any("off the commanded path" in f for f in failures)


@pytest.mark.parametrize("answer", ['{"r":{"stat":3}}', '{"r":{"stat":3},"f":[3,204,8]}', '{"r":{},"f":[3,0,8]}',
                                    '{"r":{"stat":true},"f":[3,0,8]}'])
def test_completion_poll_needs_a_valid_successful_answer(answer):
    def bad_poll(tr, data):
        if data.startswith(b'{"stat"'):
            tr.emit(answer)
        else:
            board()(tr, data)
    with pytest.raises(sender.StreamProtocolError, match="stat poll") as e:
        sender.stream(FakeTG(bad_poll), ["G1 X1 F600"], "count", timeout=5)
    assert e.value.result.acked == 1 and e.value.result.completion != "idle"


def test_bounds_hold_during_the_final_wait():
    """Motion continues after the last answer: a status outside bounds then still sends ! and %."""
    def runaway(tr, data):
        if data.startswith(b'{"stat"'):
            tr.emit('{"sr":{"posx":999,"posy":999,"stat":5}}')
            tr.emit(json.dumps({"r": {"stat": 3}, "f": [3, 0, 8]}))
        else:
            board()(tr, data)
    tg = FakeTG(runaway)
    with pytest.raises(sender.StreamRunaway) as e:
        sender.stream(tg, ["G1 X1 F600"], "count", timeout=5, bounds=((-2, 2), (-2, 2)))
    writes = [d for d, _ in tg.transport.writes]
    assert b"!" in writes and b"%" in writes
    assert e.value.result.completion != "idle"


def test_final_wait_status_keeps_metadata_and_callback():
    def late(tr, data):
        if data.startswith(b'{"stat"'):
            tr.emit('{"sr":{"posx":1.0,"posy":0.5,"stat":5}}')
            tr.emit(json.dumps({"r": {"stat": 3}, "f": [3, 0, 8]}))
        else:
            board()(tr, data)
    seen = []
    res = sender.stream(FakeTG(late), ["G1 X1 F600"], "count", timeout=5, bounds=((-2, 2), (-2, 2)),
                        on_status=lambda t, sr: seen.append(dict(sr)))
    assert res.ok
    assert res.status and "_raw" in res.status[-1][1] and "_damaged" in res.status[-1][1]
    assert seen and seen[-1]["posx"] == 1.0


def test_final_poll_can_resume_hold_without_automatic_status():
    """N78 gate: an SR list containing only line never announces HOLD to the callback."""
    held = {'value': True}

    def hold_board(tr, data):
        if data == b'~':
            held['value'] = False
        elif data.startswith(b'{"stat"'):
            stat = 6 if held['value'] else 3
            tr.emit(json.dumps({'r': {'stat': stat}, 'f': [3, 0, 8]}))
        else:
            board()(tr, data)

    tg = FakeTG(hold_board)
    seen = []

    def resume(timestamp, status):
        seen.append(status['stat'])
        if status['stat'] == 6:
            tg.transport.write(b'~')

    result = sender.stream(tg, ['G1 X1 F600'], 'g2', timeout=1.5, on_status=resume)
    assert result.ok, result
    assert [d for d, _ in tg.transport.writes].count(b'~') == 1
    assert 6 in seen
    assert result.status == [], 'stat-only polls must not create position samples'
