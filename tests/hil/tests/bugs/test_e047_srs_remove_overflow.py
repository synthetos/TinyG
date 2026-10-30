"""E047: removing items with {srs:{set:{k:f}}} writes past sr.value_flt[30].

report.c:366-372 writes sr.value_flt[i] for every working-list slot before skipping deleted
entries, and only breaks once 30 non-deleted slots are copied, so each deletion writes one more
float past the end of sr (report.h:39, :72). On a portable build the victim is st_cfg
(mt, then motor 1's ma/mi/po/pm). The damage is RAM-only; a reset reloads it from EEPROM.
"""
import pytest

from tinyg_hil.snapshot import set_status_report

pytestmark = pytest.mark.dialect("d449")     # srs is edge-only

# Settings that sit next to sr in RAM on the builds inspected so far (layout is build-dependent).
WATCH = ("mt", "1", "2", "qv", "sv", "si")


def _read(tg):
    return {t: tg.get(t) for t in WATCH}


@pytest.mark.persists
@pytest.mark.resets_board
@pytest.mark.bug("E047", on=["d449"])
def test_srs_remove_leaves_neighbours_alone(tg):
    keys = list(tg.get("sr"))
    victims = [k for k in keys if k != "stat"][:3]
    if len(victims) < 3:
        pytest.skip(f"status report list too short to remove 3 items: {keys}")
    before = _read(tg)
    try:
        tg.send_json({"srs": {"set": {k: False for k in victims}}}, check=True)
        after = _read(tg)
    finally:
        set_status_report(tg, keys)     # srs persists the list (se00-se29); put it back by name
        tg.reset()
    assert after == before
