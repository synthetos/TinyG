"""E080: in char mode a line longer than 255 characters wraps onto itself and its tail is executed,
instead of being reported as too long.

xio.h:196-197 `int size;` / `uint8_t len;`; char mode passes size 1000 (xio.c:753), so the overrun
test at xio_usart.c:278 `if (d->len >= d->size)` can never fire and xio_usart.c:292-293 wraps len to
0 after 255, overwriting the start of the line. 440 passes size 255, so its check works; 440 has no
char/line mode switch.
"""
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


@pytest.mark.persists
@pytest.mark.resets_board
@pytest.mark.bug("E080", on=["d449"])
def test_overlong_line_is_rejected(tg, keep):
    keep("rxm")
    line = '{"fb":null}' + " " * 250 + '{"fv":null}'      # 272 chars; chars 256+ are ' {"fv":null}'
    try:
        _enter_char_mode(tg)
        resp = tg.send_line(line)
        assert "fv" not in resp.body, resp.raw
        assert resp.status != 0
    finally:
        tg.reset()      # keep() then restores rxm


@pytest.mark.persists
@pytest.mark.resets_board
def test_char_mode_normal_line(tg, keep):
    keep("rxm")
    try:
        _enter_char_mode(tg)
        assert tg.send_line('{"fv":null}', check=True).body.get("fv") == pytest.approx(0.97)
    finally:
        tg.reset()
