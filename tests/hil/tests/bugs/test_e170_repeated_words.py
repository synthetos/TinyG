"""E170: reject repeated non-G/M words.

NIST RS274NGC v3 section 3.3.5 permits only one word per letter except G/M.
https://tsapps.nist.gov/publication/get_pdf.cfm?pub_id=823374
Local copy: .analysis/docs/nist-rs274ngc-v3/chapters/004_3.1-3.3_Language_Overview_Parameters_Line_Format.md
Split from test_input_regressions.py on test/gcode-input-tests.
"""
import pytest

pytestmark = pytest.mark.dialect("d449")

@pytest.mark.bug("E170", on=[])
def test_repeated_feed_word(tg):
    feed = tg.get("feed")
    try:
        assert tg.send_gcode("F100F200").status == 130
        assert tg.send_gcode("F100").status == 0
    finally:
        tg.send_gcode(f"F{feed}", check=True)
