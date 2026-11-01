"""E171: text comma end.

References: TinyG and g2core JSON-Operation (value and syntax rules), and
JSON-Details (one request per line), copied under .analysis/docs/{tinyg,g2core}-wiki/.
https://github.com/synthetos/TinyG/wiki/JSON-Operation
https://github.com/synthetos/g2/wiki/JSON-Operation
https://www.rfc-editor.org/rfc/rfc8259#section-7 (string escapes).
Split from test_input_regressions.py on test/gcode-input-tests.
The invalid read is detected with an exact-size host allocation. Serial output
alone cannot reliably distinguish the original read from a fixed parser, so no
strict-xfail board assertion is claimed for this memory-boundary case.
Text reference: .analysis/docs/g2core-wiki/Text-Mode.md; TinyG retains comma
separators as a legacy extension. This fix only bounds that normalization loop.
"""
from parser_case import ParserCase

class InputValidation(ParserCase):
    kind = 'text'

    def test_trailing_commas_stay_within_input(self):
        for data in (b"$,", b"$,,,", b"$xvm,", b"$xvm=1,000,"):
            with self.subTest(data=data):
                self.row(data)

    def test_commas_and_case_controls(self):
        row = self.row(b"$XVM=1,000")
        self.assertEqual(row[0], "0")
        self.assertEqual(float(row[3]), 1000)
