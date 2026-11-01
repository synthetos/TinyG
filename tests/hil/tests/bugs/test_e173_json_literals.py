"""E173: json literals.

References: TinyG and g2core JSON-Operation (value and syntax rules), and
JSON-Details (one request per line), copied under .analysis/docs/{tinyg,g2core}-wiki/.
https://github.com/synthetos/TinyG/wiki/JSON-Operation
https://github.com/synthetos/g2/wiki/JSON-Operation
https://www.rfc-editor.org/rfc/rfc8259#section-7 (string escapes).
Split from test_input_regressions.py on test/gcode-input-tests.
"""
import pytest

pytestmark = [pytest.mark.dialect("d449")]

@pytest.mark.bug("E173", on=["d449"])
def test_literal_tails_are_syntax_errors(tg):
    for line in ('{"ee":falsx}', '{"xvm":nonsense}'):
        assert tg.send_json(line).status == 111, line
