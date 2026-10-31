"""E154: negative F, S and out-of-range T words are accepted (RS274NGC 3.7: errors).

gcode_parser.c stores F and S as given and T as (uint8_t)trunc(value): F-100 reaches the planner,
S-100 is clamped to the speed range, T256 becomes tool 0 and T-1 tool 255. Each now fails with
nothing executed.
"""
import time

import pytest

pytestmark = [pytest.mark.motion, pytest.mark.dialect("d449")]


@pytest.mark.bug("E154", on=["d449"])
def test_negative_and_out_of_range_words_are_rejected(tg):
    start = tg.get("posx")
    tg.send_gcode("G91 G1 X0.5 F300", check=True, allow="0.5 mm at 5 mm/s")   # a valid feed is set
    tg.wait_idle()
    try:
        moved_to = tg.get("posx")
        assert tg.send_gcode("G1 X0.5 F-100", allow="must be rejected").status == tg.dialect.status("INPUT_LESS_THAN_MIN_VALUE")
        time.sleep(0.5)
        assert tg.get("posx") == pytest.approx(moved_to, abs=0.001), "the F-100 move ran"
        assert tg.send_gcode("S-100").status == tg.dialect.status("S_WORD_IS_INVALID")
        for t in ("T256", "T-1", "T1000"):
            assert tg.send_gcode(t).status == tg.dialect.status("T_WORD_IS_INVALID"), t
        assert tg.send_gcode("T5").status == 0 and tg.send_gcode("S1000").status == 0
    finally:
        tg.send_gcode(f"G90 G0 X{start:.3f}", allow="return")
        tg.wait_idle()
