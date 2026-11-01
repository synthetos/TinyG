"""E175: smart quote bounds.

References: TinyG and g2core JSON-Operation (value and syntax rules), and
JSON-Details (one request per line), copied under .analysis/docs/{tinyg,g2core}-wiki/.
https://github.com/synthetos/TinyG/wiki/JSON-Operation
https://github.com/synthetos/g2/wiki/JSON-Operation
https://www.rfc-editor.org/rfc/rfc8259#section-7 (string escapes).
Split from test_input_regressions.py on test/gcode-input-tests.
The invalid read is detected with an exact-size host allocation. Serial output
alone cannot reliably distinguish the original read from a fixed parser, so no
strict-xfail board assertion is claimed for this memory-boundary case.
"""
from parser_case import ParserCase

class InputValidation(ParserCase):
    kind = 'json'

    def test_partial_quote_stays_within_input(self):
        for data in (bytes([123, 226, 128]), bytes([123, 226]),
                     b'{"gc":"' + bytes([226, 128])):
            with self.subTest(data=data):
                self.assertNotEqual(self.row(data)[0], "0")

    def test_ascii_controls(self):
        for data in (b'{"xvm":1}', b'{"gc":"G21"}'):
            self.assertEqual(self.row(data)[0], "0")
