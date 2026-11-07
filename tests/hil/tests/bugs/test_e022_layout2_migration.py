"""Opt-in layout-1 -> layout-2 flash test; both images use the normal build.

Set TINYG_LAYOUT1_SOURCE to a layout-1 checkout and pass --allow-destructive.
Downgrading to prepare the test can reset settings; take/restore a full snapshot.
The legacy 449.01 marker is additionally covered by the deterministic host test.
"""
import os
from pathlib import Path
import re

import pytest
from tinyg_hil import flash as flasher, snapshot

SOURCE = Path(__file__).resolve().parents[4]
pytestmark = [pytest.mark.destructive, pytest.mark.resets_board,
              pytest.mark.persists, pytest.mark.dialect("d449")]


def test_layout1_upgrade_keeps_motor_geometry(tg, keep, tmp_path):
    path = os.environ.get("TINYG_LAYOUT1_SOURCE")
    if not path:
        pytest.skip("set TINYG_LAYOUT1_SOURCE to a layout-1 checkout")
    old_source = Path(path).resolve()
    config = old_source / "firmware/tinyg/config.c"
    assert re.search(r"^#define\s+CONFIG_NVM_LAYOUT\s+1\s*$", config.read_text(), re.M)
    old_hex = flasher.build(old_source)
    new_hex = flasher.build(SOURCE)
    saved = snapshot.take(tg)
    snapshot.save(saved, tmp_path)
    keep("spep", "spdp", "prbp", "spph", "spde", "coph")    # not in older dialect catalogs
    try:
        flasher.flash(tg, old_hex, expect_fb=flasher.source_build(old_source))
        restored = snapshot.restore(tg, saved)
        assert not restored["failed"], restored
        tg.set("1sa", 0.9)
        assert tg.get("1sa") == pytest.approx(0.9)
        flasher.flash(tg, new_hex, expect_fb=flasher.source_build(SOURCE))
        assert tg.get("1sa") == pytest.approx(0.9)
        assert tg.get("spep") == 1
        assert tg.get("spdp") == 0
        assert tg.get("prbp") == 0
        for token, default in (("spph", 1), ("spde", 1), ("coph", 0)):
            assert tg.get(token) == default
        tg.reset()
        assert tg.get("1sa") == pytest.approx(0.9)
    finally:
        flasher.flash(tg, new_hex, expect_fb=flasher.source_build(SOURCE))
        restored = snapshot.restore(tg, saved)
        assert not restored["failed"], restored
