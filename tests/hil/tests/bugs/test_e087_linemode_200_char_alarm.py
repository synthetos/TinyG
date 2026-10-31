"""E087: in line mode a serial line of 200+ characters raises ALARM 'Buffer full non-fatal' and the
rest of the line is then run as a separate line.

xio.h:215 RX_BUFFER_MIN_SIZE 200 and xio.c:682-687 return _FDEV_ERR on XIO_BUFFER_FULL without
freeing the FILLING header; xio_usart.c:278-283 truncates the line and clears flag_in_line, so the
next read restarts with the remainder; controller.c:218/231 call cm_alarm(STAT_BUFFER_FULL,
cs.saved_buf). 440 reads into a 255-byte buffer and accepts such lines.
"""
import time

import pytest

from tinyg_hil.protocol import Footer


@pytest.mark.bug("E087", on=[])
def test_225_char_line_does_not_alarm(tg):
    line = "G21 (" + "x" * 218 + ")"
    assert len(line) == 224
    try:
        resp = tg.send_line(line)
        later = tg.drain(quiet=0.5)
        extra = [e.raw for e in resp.events + later if e.kind in ("response", "exception", "malformed")]
        # One response (OK, or an error once overlong lines are rejected cleanly); no alarm
        # report and no second response for a leftover fragment.
        assert not extra, (resp.raw, extra)
        assert tg.stat() != tg.dialect.stat("ALARM")
    finally:
        if tg.stat() == tg.dialect.stat("ALARM"):
            tg.clear_alarm()


def test_190_char_line(tg):
    line = "G21 (" + "x" * 184 + ")"
    assert tg.send_line(line).status == 0
    assert tg.stat() != tg.dialect.stat("ALARM")


@pytest.mark.bug("E087", on=[])
def test_whitespace_only_overlong_line_gets_its_own_error(tg):
    """Review case: a valid command, a >254-space line, then recovery. The spaces line must get exactly
    one error (107) of its own; the commands around it are answered normally, with no alarm."""
    tg.drain(quiet=0.1)
    mark = len(tg.transport.history)
    tg.transport.write(b"G90\n" + b" " * 270 + b"\nG21\n")
    time.sleep(0.8)
    tg.drain(quiet=0.3)
    responses = [e for e in tg.transport.history[mark:] if e.kind == "response"]
    statuses = [Footer.from_event(e).status for e in responses]
    assert len(responses) == 3, [e.raw for e in responses]
    assert statuses.count(tg.dialect.status("INPUT_EXCEEDS_MAX_LENGTH")) == 1, [e.raw for e in responses]
    assert tg.stat() != tg.dialect.stat("ALARM")
