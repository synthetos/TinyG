"""E160: every status the firmware can report has a message, and messages name real commands."""
from pathlib import Path
import re
import unittest

ERROR_H = Path(__file__).resolve().parents[2] / "firmware/tinyg/error.h"


class StatusMessages(unittest.TestCase):
    @unittest.expectedFailure       # E160: known bug; the fix removes this marker
    def test_reported_statuses_have_messages(self):
        text = ERROR_H.read_text()
        strings = dict(re.findall(r'stat_(\d+)\[\] PROGMEM = "([^"]*)"', text))
        for code in ("27", "203", "245"):
            self.assertNotEqual(strings[code], code, f"status {code} has no message")
        self.assertNotIn("$lim", strings["203"])


if __name__ == "__main__":
    unittest.main()
