"""E168: finite word values are required before canonical execution.

NIST RS274NGC v3 section 3.3.2.1 defines signed decimal numbers, not NaN or
infinity. Source: .analysis/docs/nist-rs274ngc-v3/chapters/004_3.1-3.3_Language_Overview_Parameters_Line_Format.md
https://tsapps.nist.gov/publication/get_pdf.cfm?pub_id=823374
The firmware's BAD_NUMBER_FORMAT (102) covers invalid numeric representations,
including decimal overflow. Split from test_input_regressions.py on test/gcode-input-tests.
"""
from parser_case import ParserCase

class FiniteGcode(ParserCase):
    kind = "gcode"

    def test_nonfinite_words_never_execute(self):
        for letter in b"XYZABCIJKRPFSTLNGM":
            for number in (b"nan", b"inf", b"-inf", b"9" * 50):
                data = bytes([letter]) + number
                with self.subTest(data=data):
                    row = self.row(data)
                    self.assertEqual(row[0], "102")
                    self.assertEqual(row[1], "0", "invalid value reached execution")

    def test_finite_controls(self):
        for data in (b"G1X1.25Y-2F100", b"X.5", b"X-0", b"F0", b"S0"):
            with self.subTest(data=data):
                self.assertEqual(self.row(data)[0], "0")
