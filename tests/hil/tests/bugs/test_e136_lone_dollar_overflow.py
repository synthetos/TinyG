"""E136: a lone '$' in line mode appends "sys" past its 2-byte buffer and corrupts the next queued line.

text_parser.c:80-82 `if ((str[0] == '$') && (str[1] == NUL)) { strcat(str,"sys"); }` writes into the
line-mode buffer, which _post_buffer shrank to strlen+1 (xio.c:538); the next buffer starts one byte
later (xio.c:462). When '$' and the next line are both waiting in the USART ring, the step-2/step-3
path of _readline_linemode (xio.c:667-688) reads both before '$' is dispatched, and "sys" lands on
the start of the next line. 440 parses into a fixed 255-byte cs.in_buf.
"""
import time

import pytest

from tinyg_hil import NoResponse


def _text_until_prompts(tg, data: bytes, prompts: int, timeout=5.0) -> list[str]:
    tg.transport.write(data)
    lines, end = [], time.monotonic() + timeout
    while prompts:
        ev = tg.transport.next_event(end - time.monotonic())
        if ev is None:
            raise NoResponse(f"missing {prompts} prompt(s) after {data!r}; got {lines}")
        lines.append(ev.raw)
        prompts -= ev.kind == "prompt"
    return lines


@pytest.mark.bug("E136", on=["d449"])
def test_lone_dollar_then_query_in_one_write(tg):
    try:
        for _ in range(3):
            # The {"sys":null} reply keeps the main loop busy, so '$' and '$xvm' both sit complete
            # in the ring when the reader gets to them.
            lines = _text_until_prompts(tg, b'{"sys":null}\n$\n$xvm\n', prompts=2)
            assert any(line.startswith("[xvm]") for line in lines), lines
    finally:
        tg.get("fb")        # back to JSON mode


def test_lone_dollar_then_query_separately(tg):
    try:
        _text_until_prompts(tg, b"$\n", prompts=1)
        lines = _text_until_prompts(tg, b"$xvm\n", prompts=1)
        assert any(line.startswith("[xvm]") for line in lines), lines
    finally:
        tg.get("fb")
