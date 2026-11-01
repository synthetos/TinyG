"""E170: reject repeated non-G/M words.

NIST RS274NGC v3 section 3.3.5 permits only one word per letter except G/M.
https://tsapps.nist.gov/publication/get_pdf.cfm?pub_id=823374
Local copy: .analysis/docs/nist-rs274ngc-v3/chapters/004_3.1-3.3_Language_Overview_Parameters_Line_Format.md
Split from test_input_regressions.py on test/gcode-input-tests.
"""
from parser_case import ParserCase

class WordValidation(ParserCase):
    kind = "gcode"

    def test_invalid_words_do_not_execute(self):
        for data in tuple(bytes([c])+b"1"+bytes([c])+b"2" for c in b"XYZABCFSPTRIJKLN"):
            with self.subTest(data=data):
                row = self.row(data)
                self.assertNotEqual(row[0], "0")
                self.assertEqual(row[1], "0")

    def test_valid_controls(self):
        for data in (b"G21G90G1X1Y2F100", b"M3M8", b"X1Y2Z3", b"N1F100S10T0"):
            with self.subTest(data=data):
                self.assertEqual(self.row(data)[0], "0")
