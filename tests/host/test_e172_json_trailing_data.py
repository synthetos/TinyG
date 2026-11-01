"""E172: json trailing data.

References: TinyG and g2core JSON-Operation (value and syntax rules), and
JSON-Details (one request per line), copied under .analysis/docs/{tinyg,g2core}-wiki/.
https://github.com/synthetos/TinyG/wiki/JSON-Operation
https://github.com/synthetos/g2/wiki/JSON-Operation
https://www.rfc-editor.org/rfc/rfc8259#section-7 (string escapes).
Split from test_input_regressions.py on test/gcode-input-tests.
"""
from parser_case import ParserCase

class InputValidation(ParserCase):
    kind = 'json'

    def test_suffix_rejected_before_any_set(self):
        for suffix in (b"x", b"{}", b",", b"}", b"[]"):
            with self.subTest(suffix=suffix):
                row = self.row(b'{"xvm":1}' + suffix)
                self.assertEqual(row[0], "111")
                self.assertEqual(row[3], "0")

    def test_trailing_space_and_nested_controls(self):
        for data in (b'{"xvm":1}  ', b'{"1":{"mi":8}}', b'{"xvm":1,"ee":false}'):
            self.assertEqual(self.row(data)[0], "0")
