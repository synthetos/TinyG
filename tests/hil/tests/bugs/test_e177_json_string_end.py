"""E177: json string end.

References: TinyG and g2core JSON-Operation (value and syntax rules), and
JSON-Details (one request per line), copied under .analysis/docs/{tinyg,g2core}-wiki/.
https://github.com/synthetos/TinyG/wiki/JSON-Operation
https://github.com/synthetos/g2/wiki/JSON-Operation
https://www.rfc-editor.org/rfc/rfc8259#section-7 (string escapes).
Split from test_input_regressions.py on test/gcode-input-tests.
"""
import pytest

pytestmark = [pytest.mark.dialect("d449")]

@pytest.mark.bug("E177", on=["d449"])
def test_text_after_string_quote_is_syntax_error(tg):
    assert tg.send_json('{"gc":"G21"b}').status == 111
