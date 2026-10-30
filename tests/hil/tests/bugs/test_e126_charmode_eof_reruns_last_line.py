"""E126: at the end of a canned test the char-mode reader runs the last, already executed line again,
so M2/M30 executes twice and an extra response follows the EOF report.

xio.c:753-775 (_readline_charmode) falls through after XIO_EOF: it prints the EOF report, resets the
source and returns xio.bufp as a DEV_IS_DATA line. xio_pgm.c:44-47 fgets() writes nothing on EOF,
so the buffer still holds the last line (test_051_braid.h ends with `N2519 M30` and no newline).
Master's controller.c:214-222 returned STAT_EOF without dispatching anything.
"""
import time

import pytest

pytestmark = pytest.mark.dialect("d449")


def _enter_char_mode(tg):
    """Switch to char mode starting from fresh line-mode buffers.

    On 449.01 the first line after the switch is replaced by stale text from the base of the RX
    pool (E079). Right after a reset that text is the tokenized {"rxm":0} line itself, which only
    yields an error, so a blank line is spent on it.
    """
    tg.reset()
    tg.set("rxm", 0)
    tg.transport.write(b"\n")
    tg.drain(quiet=0.5)


def _run_canned(tg, n: int, timeout: float) -> list:
    """Start $test=n (char mode) and return every event up to and including the EOF report."""
    eof = tg.dialect.status("EOF")
    tg.send_json({"txt": f"$test={n}"}, check=True, allow=f"canned test {n}")
    events, end = [], time.monotonic() + timeout
    while True:
        ev = tg.transport.next_event(end - time.monotonic())
        assert ev is not None, f"no EOF report from $test={n} within {timeout}s"
        events.append(ev)
        if ev.kind == "exception" and ev.data["er"].get("st") == eof:
            return events


def _return_to(tg, start):
    tg.wait_idle(timeout=60)
    end = [tg.get(t) for t in ("mpox", "mpoy", "mpoz")]
    delta = " ".join(f"{ax}{s - e:.3f}" for ax, s, e in zip("XYZ", start, end))
    tg.send_gcode(f"G91 G1 {delta} F600", check=True, allow="return to the start position")
    tg.wait_idle()
    tg.send_gcode("G90", check=True)


@pytest.mark.motion
@pytest.mark.persists
@pytest.mark.resets_board
@pytest.mark.bug("E126", on=["d449"])
def test_eof_report_is_last(tg, keep):
    keep("rxm")
    start = [tg.get(t) for t in ("mpox", "mpoy", "mpoz")]
    try:
        _enter_char_mode(tg)
        events = _run_canned(tg, 51, timeout=60)      # braid: ~5 s, ends with "N2519 M30" (no newline)
        after = [e for e in tg.drain(quiet=1.5) if e.kind == "response"]
        assert not after, [e.raw for e in after]
        assert len([e for e in events if e.kind == "response" and e.body.get("n") == 2519]) == 1
    finally:
        try:
            _return_to(tg, start)       # the braid ends about (2.3, -1.5) mm from where it started
        finally:
            tg.reset()                  # clean reader state; keep() then restores rxm
