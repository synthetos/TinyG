"""E078: JSON input lines longer than 128 characters are rejected with status 107.

json_parser.h:39 `#define JSON_INPUT_STRING_MAX 128   // set an arbitrary max`, checked at
json_parser.c:129 and 235-236, while line-mode buffers hold 199 characters (xio.h:215) and 440
accepts JSON up to its 255-byte input line.
"""
import pytest

from tinyg_hil.driver import encode

pytestmark = pytest.mark.dialect("d449")     # 440 executes only the first pair of a multi-pair object

KEYS = ["posx", "posy", "posz", "posa", "mpox", "mpoy", "mpoz", "mpoa",
        "unit", "coor", "dist", "plan", "frmo", "path", "stat"]


@pytest.mark.bug("E078", on=[])
def test_long_json_get(tg):
    line = encode({k: None for k in KEYS})
    assert 128 < len(line) < 199
    resp = tg.send_line(line)
    assert resp.status == 0, resp.raw
    assert set(KEYS) <= set(resp.body)


def test_short_json_get(tg):
    line = encode({k: None for k in KEYS[:8]})
    assert len(line) <= 128
    resp = tg.send_line(line)
    assert resp.status == 0 and set(KEYS[:8]) <= set(resp.body)
