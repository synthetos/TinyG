"""E098: ChiliPeppr's 151-char status-report setup is rejected on edge, so mpox/mpoy/mpoz never arrive.

_tokenize_json_string returns STAT_INPUT_EXCEEDS_MAX_LENGTH (107) for input over 128 chars
(json_parser.c:235-236, json_parser.h:38) and SR_DEFAULTS has no mpo* (settings_default.h:84), so the
axes widget's 'Zero (G10)' sends G10 L2 P1 X0 Y0 Z0. Master has no cap and accepts the line.
"""
import pytest

from tinyg_hil import snapshot

# chilipeppr widget-tinyg widget.js:324
SR_LINE = ('{"sr":{"line":t,"posx":t,"posy":t,"posz":t,"vel":t,"unit":t,"stat":t,"feed":t,"coor":t,'
           '"momo":t,"plan":t,"path":t,"dist":t,"mpox":t,"mpoy":t,"mpoz":t}}')


@pytest.mark.persists
def test_chilipeppr_status_report_setup(tg):
    original_sr = list(tg.get("sr"))
    assert len(SR_LINE) == 151
    try:
        resp = tg.send_json(SR_LINE)
        sr = tg.get("sr")
    finally:
        if list(tg.get("sr")) != original_sr:
            snapshot.set_status_report(tg, original_sr)
    assert resp.status == 0
    assert {"mpox", "mpoy", "mpoz"} <= set(sr)
