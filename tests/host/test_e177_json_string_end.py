"""E177: json string end.

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

    def test_after_quote_requires_delimiter(self):
        for data in (b'{"gc":"a"b}', b'{"gc":"a" "b"}', b'{"gc":""x}',
                     b'{"gc":"a"1}', b'{"gc":"a"{"ee":false}}'):
            with self.subTest(data=data):
                row = self.row(data)
                self.assertEqual(row[0], "111")
                self.assertEqual(row[1], "0")

    def test_string_delimiter_controls(self):
        for data in (b'{"gc":"a" }', b'{"gc":"a", "ee":false}', b'{"gc":""}'):
            self.assertEqual(self.row(data)[0], "0")
