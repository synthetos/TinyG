"""Offline: the stream watchdog's dwell allowance for modes that cannot probe a quiet board (count, xon, rts)."""
from tinyg_hil.sender import STALL, max_dwell_s


def test_no_dwell():
    assert max_dwell_s(["G1 X1 F600", "G0 Y2", "M3 S1000"]) == 0.0


def test_dwell_seconds():
    assert max_dwell_s(["G4 P10", "G1 X1", "g04 p2.5"]) == 10.0


def test_dwell_forms():
    assert max_dwell_s(["N10 G4 P3 (pause)", "G4P1.5", "G1 X1 ; G4 P99 in a comment"]) == 3.0


def test_not_a_dwell():
    # G40 / G43 / G41 and a P word on another code are not dwells
    assert max_dwell_s(["G40 P20", "G43 H1 P30", "G41 P5", "M3 P40"]) == 0.0


def test_allowance_is_on_top_of_stall():
    assert STALL + max_dwell_s(["G4 P10"]) > 10.0


def test_compact_numbered_dwell():
    assert max_dwell_s(["N10G4P10", "n20g04p2", "N30G1X1"]) == 10.0
