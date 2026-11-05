"""Controls for the real parser/canonical/planner corpus replay tool.

These test acceptance, rejection, numeric detectors and explicit fixture gaps.
They exercise no serial ports or physical execution.
"""
import io
from pathlib import Path
import tempfile
import unittest

import corpus_replay as replay


class CorpusReplay(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="corpus-replay-test-")
        cls.addClassCleanup(cls.tmp.cleanup)
        cls.root = Path(cls.tmp.name)
        cls.exe = replay.build(replay.HERE.parents[1], cls.root / "build")
        (cls.root / "logs").mkdir()

    def run_lines(self, content):
        path = self.root / "case.nc"
        path.write_bytes(content)
        events = io.StringIO()
        row = replay.replay(self.exe, path, self.root, 0, 15, events)
        self.assertEqual(row["exit"], 0, row.get("stderr"))
        self.assertTrue(row["complete"])
        return row, events.getvalue()

    def test_real_chain_and_detector_controls(self):
        replay.self_test(self.exe)

    def test_settings_coverage_is_distinct_from_unknown_name(self):
        row, events = self.run_lines(b'{"ct":0.01}\n$HZ\n$H\n')
        self.assertEqual(row["statuses"], {"1": 100, "2": 100})
        self.assertEqual([(x["type"], x["line"]) for x in row["findings"]],
                         [("UNMODELED_CONFIG", 1)])
        self.assertEqual(row["report_stubs"], [3])
        self.assertIn('"text": "{\\"ct\\":0.01}"', events)

    def test_line_numbers_and_program_sync(self):
        row, _ = self.run_lines(b'G1 X1\r\nM400\nG1 X2\nG1 X1X2')
        self.assertEqual(row["lines"], 4)
        self.assertEqual(row["blocks"], 2)
        self.assertEqual(row["statuses"], {"4": 130})
        self.assertFalse(row["findings"])
        self.assertFalse(row["alarms"])

    def test_empty_input(self):
        row, _ = self.run_lines(b'')
        self.assertEqual(row["lines"], 0)
        self.assertEqual(row["blocks"], 0)

    def test_float_conversion_preserves_comments_and_strings(self):
        source = 'float x = sqrt(2.0) + 1e-3; /* sqrt(2.0) */ "sin(1.0)";'
        self.assertEqual(replay.binary32(source),
                         'float x = sqrtf(2.0f) + 1e-3f; /* sqrt(2.0) */ "sin(1.0)";')


if __name__ == "__main__":
    unittest.main()
