"""E106: {"st":2} answers OK (and persists 2) because sw_set_st drops set_01's range error.

switch.c:291-296 `set_01(nv); switch_init(); return (STAT_OK);` ignores config.c:356-357, so the
parsers persist 2 (json_parser.c:156-158) and st reloads as 0 after a reset. Master switch.c:233-238 is the same.
"""
import pytest


@pytest.mark.persists
@pytest.mark.bug("E106", on=["d440", "d449"])
def test_st_out_of_range_rejected(tg):
    original = tg.get("st")
    try:
        resp = tg.send_json({"st": 2})
        assert resp.status == tg.dialect.status("INPUT_VALUE_RANGE_ERROR")
        assert tg.get("st") == original
    finally:
        # The live value never changes, so keep() would not notice the 2 that stock firmware persisted.
        tg.set("st", original, allow="restore persisted st")
