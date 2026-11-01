"""E172: json trailing data.

References: TinyG and g2core JSON-Operation (value and syntax rules), and
JSON-Details (one request per line), copied under .analysis/docs/{tinyg,g2core}-wiki/.
https://github.com/synthetos/TinyG/wiki/JSON-Operation
https://github.com/synthetos/g2/wiki/JSON-Operation
https://www.rfc-editor.org/rfc/rfc8259#section-7 (string escapes).
Split from test_input_regressions.py on test/gcode-input-tests.
"""
import pytest

pytestmark = [pytest.mark.dialect("d449"), pytest.mark.persists]

@pytest.mark.bug("E172", on=[])
def test_suffix_does_not_apply_setting(tg, keep):
    keep("xvm")
    before = tg.get("xvm")
    value = 123 if before != 123 else 124
    response = tg.send_json('{"xvm":%d}x' % value)
    assert response.status == 111
    assert tg.get("xvm") == before
