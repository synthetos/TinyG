"""E042: any unsupported G-code or unknown word (G43 Hn, G41, G98, O-number, D) raises ALARM; master only returned an error.

edge gcode_parser.c:423 `default: return (cm_alarm(STAT_GCODE_COMMAND_UNSUPPORTED, cs.saved_buf));` for
unknown letters and :427-428 for unsupported G numbers; afterwards :72 rejects every block with 204.
Master returns STAT_GCODE_COMMAND_UNSUPPORTED as a plain error (master gcode_parser.c:401, :405).
Arc specification errors alarm too (plan_arc.c:216/240/299/329); those are covered with motion in E054/E074.
"""
import pytest


@pytest.mark.parametrize("line", ["G43 H1", "G41", "G98", "O1001", "G0 D1"])
@pytest.mark.bug("E042", on=["d449"])
def test_unsupported_word_is_an_error_not_an_alarm(tg, line):
    tg.send_gcode(line)
    assert tg.stat() != tg.dialect.stat("ALARM")
    assert tg.send_gcode("G90").status == 0

