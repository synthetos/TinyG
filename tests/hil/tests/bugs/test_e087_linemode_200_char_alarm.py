"""E087: in line mode a serial line of 200+ characters raises ALARM 'Buffer full non-fatal' and the
rest of the line is then run as a separate line.

xio.h:215 RX_BUFFER_MIN_SIZE 200 and xio.c:682-687 return _FDEV_ERR on XIO_BUFFER_FULL without
freeing the FILLING header; xio_usart.c:278-283 truncates the line and clears flag_in_line, so the
next read restarts with the remainder; controller.c:218/231 call cm_alarm(STAT_BUFFER_FULL,
cs.saved_buf). 440 reads into a 255-byte buffer and accepts such lines.
"""
import pytest


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
