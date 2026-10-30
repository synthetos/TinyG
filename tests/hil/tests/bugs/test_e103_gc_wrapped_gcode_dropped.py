"""E103: {"gc":"..."} is silently discarded, and {"srs":{"clear":false}} clears the status-report list.

The gc row is typed _n0 (config_app.c:499), so the typesafe marshaller links no string
(json_parser.c:437, 461-463, 485) and gc_run_gc gets nv->str == NULL (config.c:659,
gcode_parser.c:61-62, 565-567). _n0 children also keep value_int = 0xFFFFFFFF, which
_set_srs reads as "clear" (report.c:295-296).
"""
import pytest

from tinyg_hil.snapshot import set_status_report


def test_gc_wrapped_gcode_runs(tg):
    tg.send_gcode("G21", check=True)
    try:
        tg.send_json({"gc": "G20"}, check=True)
        assert tg.get("unit") == 0
    finally:
        tg.send_gcode("G21", check=True)


@pytest.mark.dialect("d449")
@pytest.mark.persists
def test_srs_clear_false_keeps_list(tg):
    keys = list(tg.get("sr") or {})
    try:
        tg.send_json({"srs": {"clear": False}}, check=True)
        assert list(tg.get("sr") or {}) == keys
    finally:
        set_status_report(tg, keys)


@pytest.mark.dialect("d449")
@pytest.mark.parametrize("value", [False, 1])
def test_null_only_command_rejects_values(tg, value):
    assert tg.send_json({"er": value}).status != 0


@pytest.mark.dialect("d449")
def test_gc_null_remains_a_query(tg):
    tg.send_json({"gc": None}, check=True)
