"""N74 (GitHub #255): no tracked file may carry merge conflict markers.

Unresolved markers were committed in the AVR Studio 4 build files (firmware/tinyg/default/Makefile, tinyg.aps,
support/tinyg.aws), in support/tinyg.cproj that support/tinyg.atsln opened, and in support/readme_master.md; the
default/ Makefile failed to run ("missing separator"). Lines that are exactly a conflict marker are refused; '======='
on its own counts only inside a <<<<<<< ... >>>>>>> block, since Markdown headings underline with it.
"""
from pathlib import Path
import re, subprocess, unittest
ROOT = Path(__file__).resolve().parents[2]


class NoMergeMarkers(unittest.TestCase):
    def test_no_conflict_markers(self):
        files = subprocess.run(['git', '-C', str(ROOT), 'ls-files', '-z'], capture_output=True, check=True).stdout
        bad = []
        for name in files.decode().split('\0'):
            p = ROOT / name
            if not name or not p.is_file() or p.stat().st_size > 4_000_000:
                continue
            data = p.read_bytes()
            if b'\0' in data[:8192]:
                continue                                # binary
            for n, line in enumerate(data.decode('latin-1').splitlines(), 1):
                if re.match(r'^(<{7}|>{7})( |$)', line):
                    bad.append(f'{name}:{n}: {line[:40]}')
        self.assertEqual(bad, [], 'merge conflict markers:\n' + '\n'.join(bad[:20]))

    def test_solution_opens_the_firmware_project(self):
        sln = (ROOT / 'support/tinyg.atsln').read_text(errors='replace')
        m = re.search(r'"tinyg", "([^"]+\.cproj)"', sln)
        self.assertIsNotNone(m, 'support/tinyg.atsln names no tinyg project')
        proj = (ROOT / 'support' / m.group(1).replace('\\', '/')).resolve()
        self.assertTrue(proj.is_file(), f'support/tinyg.atsln opens {m.group(1)}, which does not exist')
        self.assertEqual(proj, (ROOT / 'firmware/tinyg/tinyg.cproj').resolve())


if __name__ == '__main__':
    unittest.main()
