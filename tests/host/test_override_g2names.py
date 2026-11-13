"""Public override settings are ordinary RAM-only configuration entries."""
from pathlib import Path
import re
import unittest

FW=Path(__file__).resolve().parents[2]/'firmware/tinyg'

class OverrideNames(unittest.TestCase):
    def test_public_rows_are_readable_in_text_and_never_initialized_or_persisted(self):
        source=(FW/'config_app.c').read_text()
        rows={m[1]:m[2].split(',') for m in re.finditer(
            r'\{\s*""\s*,\s*"([^"]+)"\s*,([^}\n]+)',source)}
        for token in ('mfo','mfoe','m48e','mfc','mcp','mfe'):
            with self.subTest(token=token):
                row=[v.strip() for v in rows[token]]
                self.assertIn(row[0],('_f0','_i0'))
                self.assertNotEqual(row[2],'tx_print_nul')
                if token in ('mfo','mfoe','m48e'):
                    self.assertNotEqual(row[4],'set_nul')
                else:self.assertEqual(row[4],'set_nul')
        # These flags are the actual persistence/default-init contract.
        header=(FW/'config.h').read_text()
        for flag in ('_f0','_i0'):
            definition=re.search(r'#define\s+'+flag+r'\s+([^\n]+)',header)[1]
            self.assertNotIn('F_PERSIST',definition)
            self.assertNotIn('F_INITIALIZE',definition)
