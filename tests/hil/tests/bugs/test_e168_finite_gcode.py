"""E168: reject nonfinite feed words without motion.

NIST RS274NGC v3 section 3.3.2.1 defines signed decimal numbers.
https://tsapps.nist.gov/publication/get_pdf.cfm?pub_id=823374
The host test covers axes, offsets, parameters, G/M and integer words as well.
"""
import pytest

pytestmark = pytest.mark.dialect("d449")

@pytest.mark.bug("E168", on=["d449"])
def test_nonfinite_feed_is_number_format_error(tg):
    feed = tg.get("feed")
    try:
        for line in ("Fnan", "Finf", "F-inf", "F" + "9" * 50):
            assert tg.send_gcode(line).status == 102, line
    finally:
        tg.send_gcode(f"F{feed}", check=True)
