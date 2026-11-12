"""Offline: the file soak's host-side line filter (tinyg_hil.hostlines, N67/N101)."""
from tinyg_hil.hostlines import host_safe


def send(lines):
    return host_safe(lines)[0]


def test_text_mode_and_flush_lines_are_dropped():
    assert send(["$HZ", "  $X", "?", "h", "H", "%", "% ", " %\t"]) == []


def test_realtime_chars_are_removed_before_classifying():
    # removing ! or ~ must not leave a dangerous line behind
    assert send(["!$HZ", "~?", "!%", "~h", "!!"]) == []


def test_ordinary_gcode_and_comments_stay_intact():
    lines = ["G21 G90", "(Pen tool)", "G1 X1.5 Y-2 F900", "M3 S1000", "; comment with % inside", "G2 X1 Y1 I1 J0"]
    assert send(lines) == lines


def test_realtime_chars_inside_comments_are_removed():
    assert send(["(careful! resume~ here)", "G1 X1 ;stop!"]) == ["(careful resume here)", "G1 X1 ;stop"]


def test_record_keeps_the_original_text():
    _, rec = host_safe(["G21", "!$HZ", "(a!)", "M30"])
    assert rec["dropped"] == [(2, "!$HZ")] and rec["dropped_count"] == 1
    assert rec["realtime_removed"] == [(2, "!$HZ"), (3, "(a!)")] and rec["realtime_removed_count"] == 2


def test_raw_keeps_the_file_unchanged():
    lines = ["$HZ", "!", "G1 X1"]
    out, rec = host_safe(lines, raw=True)
    assert out == lines and rec == {"raw": True}
