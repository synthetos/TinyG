"""E174: json value spacing.

References: TinyG and g2core JSON-Operation (value and syntax rules), and
JSON-Details (one request per line), copied under .analysis/docs/{tinyg,g2core}-wiki/.
https://github.com/synthetos/TinyG/wiki/JSON-Operation
https://github.com/synthetos/g2/wiki/JSON-Operation
https://www.rfc-editor.org/rfc/rfc8259#section-7 (string escapes).
Split from test_input_regressions.py on test/gcode-input-tests.
"""
import pytest

pytestmark = [pytest.mark.dialect("d449"), pytest.mark.persists]

@pytest.mark.bug("E174", on=["d449"])
def test_separated_digits_do_not_set(tg, keep):
    keep("xvm")
    before = tg.get("xvm")
    assert tg.send_json('{"xvm":1 2}').status == 111
    assert tg.get("xvm") == before
