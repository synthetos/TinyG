"""Shared runner for the split parser regressions; execution services are stubbed."""
from pathlib import Path
import tempfile
import unittest
from input_harness import build, run

class ParserCase(unittest.TestCase):
    kind = None

    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory(prefix="tinyg-parser-")
        cls.addClassCleanup(cls.directory.cleanup)
        cls.exe = build(cls.kind, Path(cls.directory.name))

    def row(self, data):
        result = run(self.exe, [data])
        self.assertEqual(result.returncode, 0, result.stderr)
        rows = result.stdout.splitlines()
        self.assertEqual(len(rows), 1, result.stdout)
        return rows[0].split()
