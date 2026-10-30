"""E123: $test=N does nothing in the default line mode ($rxm=1) and leaves the PGM file armed.

test.c:62-84 opens the PGM file and calls controller_set_primary_source(XIO_DEV_PGM), but the
line-mode reader is hard-wired to USB (xio.c:668 and 682 `xio_gets_usart(&ds[XIO_DEV_USB], ...)`);
only the char-mode reader reads xio.primary_src (xio.c:753). Line mode is the default
(settings_default.h:74). 440 has a single reader that honours the primary source.
"""
import pytest

pytestmark = pytest.mark.dialect("d449")


@pytest.mark.motion
@pytest.mark.resets_board
@pytest.mark.bug("E123", on=["d449"])
def test_canned_test_runs_in_line_mode(tg):
    if tg.get("rxm") != 1:
        pytest.skip("needs the default line mode ($rxm=1)")
    run = tg.dialect.stat("RUN")
    try:
        # Test 51 (braid) is a few seconds of sub-millimetre moves; the reset below stops it.
        tg.send_json({"txt": "$test=51"}, check=True, allow="canned test 51, stopped by ^X")
        later = tg.drain(quiet=3.0, limit=4.0)
        activity = [e.raw for e in later
                    if e.kind == "response" or (e.kind == "status" and e.body.get("stat") == run)]
        assert activity, "no line of the canned test was executed"
    finally:
        tg.reset()      # also disarms the PGM file left as primary source
