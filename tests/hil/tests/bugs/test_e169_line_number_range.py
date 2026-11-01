"""E169: validate line numbers before unsigned conversion.

NIST RS274NGC v3 section 3.3.1 defines nonnegative integer line labels.
https://tsapps.nist.gov/publication/get_pdf.cfm?pub_id=823374
Local copy: .analysis/docs/nist-rs274ngc-v3/chapters/004_3.1-3.3_Language_Overview_Parameters_Line_Format.md
Split from test_input_regressions.py on test/gcode-input-tests.
This bounded fix preserves TinyG's existing wider uint32 line-number range;
it does not impose NIST's five-digit limit or change word placement.
"""
import pytest

pytestmark = pytest.mark.dialect("d449")

@pytest.mark.bug("E169", on=[])
def test_invalid_line_number(tg):
    for line in ("N-1", "N1.5", "N4294967296"):
        assert tg.send_gcode(line).status != 0, line
    assert tg.send_gcode("N99999").status == 0
