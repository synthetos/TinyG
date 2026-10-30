"""E079: changing $rxm at runtime drops or replays buffered input; nothing is re-initialised.

config_app.c:452 binds rxm to plain set_01. Line mode keeps a pre-allocated FILLING buffer with
ds[USB].flag_in_line set, so after {"rxm":0} the char-mode reader (xio.c:753) keeps filling that
old line-mode buffer (xio_usart.c:300-306) while it parses the stale pool base (xio.c:723) instead
of the line it just read. xio_reset_readline_linemode() (xio.c:407) has no callers. 440 has no rxm.
"""
import time

import pytest

pytestmark = pytest.mark.dialect("d449")


def _replies(tg, data: bytes) -> list[tuple]:
    """Write several lines at once; return (first body key, status) per response, or the raw line."""
    tg.transport.write(data)
    out = []
    for e in tg.drain(quiet=1.0):
        if e.kind == "response" and e.body:
            out.append((next(iter(e.body)), e.data["f"][1]))
        elif e.kind in ("response", "exception", "malformed"):
            out.append(e.raw)
    return out


@pytest.mark.persists
@pytest.mark.resets_board
@pytest.mark.bug("E079", on=["d449"])
def test_rxm_switch_keeps_following_lines(tg, keep):
    keep("rxm")             # restored after the reset below
    if tg.get("rxm") != 1:
        tg.set("rxm", 1)
    # Fresh buffers: the stale text the char-mode reader then picks up is the tokenized
    # {"rxm":0} line, which only yields an error instead of re-running an old G-code line.
    tg.reset()
    try:
        assert _replies(tg, b'{"rxm":0}\n{"fb":null}\n{"fv":null}\n') == [("rxm", 0), ("fb", 0), ("fv", 0)]
        time.sleep(0.2)
        assert _replies(tg, b'{"rxm":1}\n{"fb":null}\n') == [("rxm", 0), ("fb", 0)]
    finally:
        tg.reset()          # discard whatever stale reader state the switches left behind
