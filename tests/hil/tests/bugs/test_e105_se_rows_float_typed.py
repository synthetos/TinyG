"""E105: se00-se29 are typed float but use set_u16, so writing back a slot's own index stores 0 (fb).

config_app.c:591-620 `_fp ... get_u16, set_u16`; JSON puts N.0 in value_flt (json_parser.c:439-442)
and set_u16 keeps the low half of its bit pattern (config.c:344), which is 0 for small integers.
"""
import struct

import pytest

from tinyg_hil.snapshot import set_status_report


def _stored_as(index: int) -> int:
    """What stock 449 set_u16 stores for a float-typed write of index."""
    return struct.unpack("<I", struct.pack("<f", float(index)))[0] & 0xFFFF


@pytest.mark.persists
def test_se_slot_round_trips(tg):
    keys = list(tg.get("sr") or {})
    index = tg.get("se00")
    if _stored_as(index) != 0:
        pytest.skip(f"writing {index} would store row {_stored_as(index)} on stock firmware (E140 restart hazard)")
    try:
        tg.set("se00", index)
        assert tg.get("se00") == index
        assert list(tg.get("sr") or {}) == keys
    finally:
        set_status_report(tg, keys)
