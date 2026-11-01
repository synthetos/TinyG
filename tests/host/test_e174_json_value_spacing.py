"""E174: json value spacing.

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

    def test_whitespace_does_not_join_value_parts(self):
        for data in (b'{"xvm":1 2}', b'{"xvm":1 .2}', b'{"xvm":1e 2}',
                     b'{"ee":t rue}', b'{"xvm":n ull}'):
            with self.subTest(data=data):
                row = self.row(data)
                self.assertEqual(row[0], "111")
                self.assertEqual(row[1], "0")

    def test_structural_spaces_and_string_spaces(self):
        for data in (b' { "xvm" : 12 } ', b'{xvm:1e2, ee:false}', b'{"gc":"a b"}'):
            self.assertEqual(self.row(data)[0], "0")
