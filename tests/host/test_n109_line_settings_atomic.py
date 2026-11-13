"""Real JSON parser and N60 guard: inspect the whole line before any SET."""
import tempfile
import unittest

from input_harness import FIXTURES, build, run


class WholeLineSettings(unittest.TestCase):
    def test_busy_line_and_group_are_atomic(self):
        pre = (FIXTURES / 'json_prelude.h').read_text()
        pre = pre.replace('bool mp_runtime_is_idle(void) { return true; }',
                          'bool mp_runtime_is_idle(void) { return false; }')
        pre = pre.replace('(void)i; (void)mask; return false;',
                          'return (i == 1 || i == 2) && (mask & F_PERSIST);')
        pre = pre.replace('"ee"', '"n"').replace('TYPE_NULL, TYPE_BOOLEAN,', 'TYPE_NULL, TYPE_INTEGER,')
        cases = [
            (b'{"xvm":123}', 117, 0),
            (b'{"n":12345,"xvm":123}', 117, 0),
            (b'{"xvm":123,"n":12345}', 117, 0),
            (b'{"n":12345,"1":{"mi":4}}', 117, 0),
            (b'{"n":12345,"xvm":null}', 0, 1),
            (b'{"n":12345}', 0, 1),
            (b'{"txt":"{\\"n\\":12345,\\"xvm\\":123}"}', 117, 0),
        ]
        with tempfile.TemporaryDirectory(prefix='tinyg-n109-') as folder:
            exe = build('json', folder, prelude=pre)
            result = run(exe, [c[0] for c in cases])
        self.assertEqual(result.returncode, 0, result.stderr)
        rows = result.stdout.splitlines()
        self.assertEqual(len(rows), len(cases))
        for (line, status, sets), row in zip(cases, rows):
            with self.subTest(line=line):
                values = row.split()
                self.assertEqual(int(values[0]), status, row)
                self.assertEqual(int(values[3]), sets, row)
