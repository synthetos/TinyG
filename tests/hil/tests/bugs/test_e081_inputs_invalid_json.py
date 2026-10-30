"""E081: reading inputs ({"in1":null}, {"in":null}) returns invalid JSON.

util.c:333-337 strcat_signed() formats with PSTR("%l"), which has no conversion character, so
avr-libc prints nothing. json_parser.c:575 uses it for TYPE_SIGNED, which get_int sets for
in1..in8 (config.c:303-308, config_app.c:484-491): the reply reads '"in1":' with no value.
"""
import re

import pytest

pytestmark = pytest.mark.dialect("d449")     # 440 has no inN tokens


def _send_any(tg, line, timeout=2.0):
    """Like send_line, but also returns a JSON line that fails to parse instead of timing out."""
    tg.transport.write(line.encode() + b"\n")
    return tg.wait_for(lambda e: e.kind in ("response", "malformed"), timeout)


def test_read_single_input(tg):
    ev = _send_any(tg, '{"in1":null}')
    assert ev.kind == "response", ev.raw
    assert isinstance(ev.data["r"]["in1"], int)


def test_read_input_group(tg):
    ev = _send_any(tg, '{"in":null}')
    assert ev.kind == "response", ev.raw
    group = ev.data["r"]["in"]
    assert len(group) == 8 and all(isinstance(v, int) for v in group.values())


def test_read_input_text_mode(tg):
    # sw_print_in has its own "%2d" format; the token may print with or without its group
    assert any(re.match(r"Input (in)?1 state:\s+-?\d+$", line) for line in tg.send_text("$in1"))
