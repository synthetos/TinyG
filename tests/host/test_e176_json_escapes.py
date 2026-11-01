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
from parser_case import ParserCase

class InputValidation(ParserCase):
    kind = 'json'

    def test_control_escapes_decode(self):
        import json
        for value in ("a\tb", "a\nb", "a\rb", "a\bb", "a\fb", 'a"b', "a\\b", "a/b"):
            data = json.dumps({"gc": value}).encode()
            with self.subTest(value=value):
                row = self.row(data)
                self.assertEqual(row[0], "0")
                self.assertEqual(bytes.fromhex(row[8]), value.encode())
        self.assertEqual(bytes.fromhex(self.row(br'{"gc":"a\/b"}')[8]), b'a/b')

    def test_invalid_escape_rejected(self):
        self.assertEqual(self.row(br'{"gc":"a\qb"}')[0], "111")
