"""E176: json escapes.

References: TinyG and g2core JSON-Operation (value and syntax rules), and
JSON-Details (one request per line), copied under .analysis/docs/{tinyg,g2core}-wiki/.
https://github.com/synthetos/TinyG/wiki/JSON-Operation
https://github.com/synthetos/g2/wiki/JSON-Operation
https://www.rfc-editor.org/rfc/rfc8259#section-7 (string escapes).
Split from test_input_regressions.py on test/gcode-input-tests.
Only the single-character JSON escapes are covered; unsupported escapes must
be rejected rather than silently changing a command.
"""
import pytest

pytestmark = [pytest.mark.dialect("d449")]

@pytest.mark.bug("E176", on=[])
def test_message_tab_escape(tg):
    response = tg.send_json({"gc": "(msgA\tB)"})
    assert response.status == 0
    assert response.body.get("msg") == "A\tB"
