"""E160: small validation and message gaps.

gcode_parser.c stores L into a uint8: G10 L258 and G10 L2.5 both ran as L2 and rewrote G54. qv
accepted 3 (valid 0-2). bjd and cjd reported with 0 decimals (ajd uses 4).
"""
import pytest

pytestmark = [pytest.mark.persists, pytest.mark.dialect("d449")]


def test_l_word_and_qv_are_validated(tg, keep):
    keep("g54x", "qv")
    before = tg.get("g54x")
    for line in ("G10 L258 P1 X7", "G10 L2.5 P1 X7", "G10 L-2 P1 X7"):
        assert tg.send_gcode(line).status == tg.dialect.status("L_WORD_IS_INVALID"), line
    assert tg.get("g54x") == pytest.approx(before), "a wrapped L rewrote G54"
    qv = tg.get("qv")
    assert tg.send_json({"qv": 3}).status != 0
    assert tg.get("qv") == qv


def test_b_c_junction_deviation_keep_decimals(tg):
    for token in ("bjd", "cjd"):
        tg.transport.write(('{"%s":null}\n' % token).encode())
        raw = [e.raw for e in tg.drain(quiet=0.5) if f'"{token}"' in e.raw]
        assert raw and f'"{token}":0.' in raw[0], raw
