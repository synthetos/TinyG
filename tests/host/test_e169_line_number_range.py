"""E169: validate line numbers before unsigned conversion.

NIST RS274NGC v3 section 3.3.1 defines nonnegative integer line labels.
https://tsapps.nist.gov/publication/get_pdf.cfm?pub_id=823374
Local copy: .analysis/docs/nist-rs274ngc-v3/chapters/004_3.1-3.3_Language_Overview_Parameters_Line_Format.md
Split from test_input_regressions.py on test/gcode-input-tests.
This bounded fix preserves TinyG's existing wider uint32 line-number range;
it does not impose NIST's five-digit limit or change word placement.
"""
from parser_case import ParserCase

class WordValidation(ParserCase):
    kind = "gcode"

    def test_invalid_words_do_not_execute(self):
        for data in (b"N-1", b"N1.5", b"N1.0001", b"N4294967296", b"Nnan", b"Ninf"):
            with self.subTest(data=data):
                row = self.row(data)
                self.assertNotEqual(row[0], "0")
                self.assertEqual(row[1], "0")

    def test_valid_controls(self):
        for data in (b"N0", b"N99999", b"N100000", b"N4294967040", b"N1G21"):
            with self.subTest(data=data):
                self.assertEqual(self.row(data)[0], "0")
