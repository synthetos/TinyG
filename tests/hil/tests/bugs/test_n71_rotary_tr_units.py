"""N71: $Ntr of a motor mapped to a rotary axis is degrees per revolution, in G20 as in G21.

The $Ntr rows carry F_CONVERT (config_app.c) and st_set_tr() stores through set_flu() (stepper.c), so a value set
while the board is in G20 was multiplied by 25.4 for every motor; prep_float() divided it back for display. For a
motor mapped to A, B or C the travel per revolution is in degrees, so $4tr=5 sent in G20 was stored as 127 and that
rotary moved 25.4 times short, while every readout in G20 still showed 5 (GitHub #139). Wiki Additional-Notes: for a
rotary, travel per revolution is the degrees the machine moves per motor revolution (90:1 table -> 4); wiki
Inch-and-Millimeter-Units-Mode lists $Ntr among the settings G20 converts, which holds for linear motors.

No motion: motor 4 is mapped to A (degrees), $4tr is set in G20 and read back in G21; a linear motor still converts.
"""
import pytest

pytestmark = [pytest.mark.persists, pytest.mark.dialect("d449"), pytest.mark.timeout(120),
              pytest.mark.bug("N71", on=[])]


def test_rotary_motor_tr_is_not_unit_converted(tg, keep, record):
    keep("4ma", "4tr", "aam", "1tr")
    tg.send_gcode("G21", check=True)
    tg.set("aam", 1, allow="A in degrees")
    tg.set("4ma", 3, allow="motor 4 drives A")
    linear_mm = tg.get("1tr")
    rows = {}
    try:
        tg.send_gcode("G20", check=True)
        tg.set("4tr", 5, allow="rotary travel per revolution, set in G20")
        rows["4tr_g20"] = tg.get("4tr")
        tg.set("1tr", round(linear_mm / 25.4, 6), allow="linear travel per revolution, set in G20")
        tg.send_gcode("G21", check=True)
        rows["4tr_g21"] = tg.get("4tr")
        rows["1tr_g21"] = tg.get("1tr")
    finally:
        tg.send_gcode("G21", check=True)
    record("n71", {"linear_mm": linear_mm, **rows})
    print("N71", linear_mm, rows)
    assert rows["4tr_g21"] == pytest.approx(5, abs=1e-3), f"rotary $4tr set as 5 in G20 was stored as {rows['4tr_g21']}"
    assert rows["4tr_g20"] == pytest.approx(5, abs=1e-3), f"rotary $4tr reads {rows['4tr_g20']} in G20"
    assert rows["1tr_g21"] == pytest.approx(linear_mm, rel=1e-4), "a linear motor's $Ntr must still convert from inches"
