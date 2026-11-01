"""E173: json literals.

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

    def test_literal_tails_rejected(self):
        for data in (b'{"ee":trux}', b'{"ee":falsx}', b'{"xvm":nonsense}',
                     b'{"sr":{"unknown":trux}}', b'{"xvm":1,"ee":truejunk}'):
            with self.subTest(data=data):
                row = self.row(data)
                self.assertEqual(row[0], "111")
                self.assertEqual(row[1], "0")

    def test_documented_shorthand_controls(self):
        for literal in (b"true", b"t", b"false", b"f", b"TRUE", b"FALSE"):
            self.assertEqual(self.row(b'{ee:'+literal+b'}')[0], "0")
        for literal in (b"null", b"n", b"NULL", b'""'):
            self.assertEqual(self.row(b'{xvm:'+literal+b'}')[0], "0")
