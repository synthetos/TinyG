"""N104: exercise actual parser, arc entry, coordinate conversion and planner.

LinuxCNC G-codes 5.1 Center Format Arcs (including the G90.1 both-word rule):
https://linuxcnc.org/docs/html/gcode/g-code.html#_center_format_arcs
NIST RS274NGC v3 3.5.3.2:
https://tsapps.nist.gov/publication/get_pdf.cfm?pub_id=823374
"""
from pathlib import Path
import json
import subprocess
import tempfile
import unittest

import corpus_replay as replay

ROOT = Path(__file__).resolve().parents[2]
PLANES = (("G17", "X", "Y", "I", "J", "K"),
          ("G18", "Z", "X", "K", "I", "J"),
          ("G19", "Y", "Z", "J", "K", "I"))


class ArcMissingOffsets(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.tmp.cleanup)
        out = Path(cls.tmp.name)
        cls.exe = replay.build(ROOT, out)
        path = out / 'driver.c'
        driver = path.read_text()
        marker = '        if (cm_is_alarmed() != STAT_OK) printf("STATE'
        assert driver.count(marker) == 1
        driver = driver.replace(marker,
                                '        printf("TARGET %u %.9g %.9g %.9g\\n", source_line, '
                                '(double)cm.gm.target[0], (double)cm.gm.target[1], '
                                '(double)cm.gm.target[2]);\n' + marker)
        path.write_text(driver)
        subprocess.run(json.loads((out / 'command.json').read_text()),
                       check=True, capture_output=True, text=True)

    def run_lines(self, lines):
        result = subprocess.run([str(self.exe)], input='\n'.join(lines) + '\n',
                                capture_output=True, text=True, timeout=15, check=True)
        self.assertFalse(result.stderr, result.stderr)
        states, targets = {}, {}
        for line in result.stdout.splitlines():
            fields = line.split()
            if fields[0] == 'LINE':
                states[int(fields[1])] = (int(fields[2]), int(fields[3]),
                                           tuple(map(float, fields[5:8])))
            elif fields[0] == 'TARGET':
                targets[int(fields[1])] = tuple(map(float, fields[2:5]))
        self.assertEqual(len(states), len(lines), result.stdout)
        return states, targets

    def check_refusal(self, setup, bad, following):
        lines = setup + [bad, following]
        states, targets = self.run_lines(lines)
        rejected = len(lines) - 1
        self.assertEqual(states[rejected][0], 157, (bad, states))
        self.assertNotEqual(states[rejected][1], 2, (bad, states))
        self.assertEqual(states[rejected][2], states[rejected - 1][2], bad)
        self.assertEqual(targets[rejected], targets[rejected - 1], bad)
        self.assertEqual(states[rejected + 1][0], 0, (following, states))

    def test_missing_and_resolved_zero_centres_do_not_change_the_target(self):
        for plane, a, b, i, j, normal in PLANES:
            for mode in ('G91.1', 'G90.1'):
                for direction in ('G2', 'G3'):
                    setup = [f'G21 G90 G94 {plane} {mode}', f'G1 {a}5 {b}6 F600']
                    offsets = f'{i}0 {j}0' if mode == 'G91.1' else f'{i}5 {j}6'
                    for words in ('', offsets, f'{normal}1', offsets + f' {normal}1'):
                        with self.subTest(plane=plane, mode=mode, direction=direction, words=words):
                            self.check_refusal(setup, f'{direction} {a}1 {b}1 {words}', f'G1 {a}0 {b}0')

    def test_absolute_zero_words_can_still_describe_a_valid_centre(self):
        for plane, a, b, i, j, _ in PLANES:
            for direction in ('G2', 'G3'):
                for end in (f'{a}0 {b}5', ''):  # a quarter arc or a full circle
                    with self.subTest(plane=plane, direction=direction, end=end):
                        states, _ = self.run_lines([f'G21 G90 G94 {plane} G90.1',
                                                   f'G1 {a}5 F600', f'{direction} {end} {i}0 {j}0'])
                        self.assertTrue(all(s[0] == 0 and s[1] != 2 for s in states.values()), states)

    def test_single_incremental_word_is_valid_but_absolute_needs_both(self):
        for plane, a, b, i, j, _ in PLANES:
            setup = [f'G21 G90 G94 {plane} G90.1', f'G1 {a}5 F600']
            self.check_refusal(setup, f'G2 {a}0 {b}5 {i}0', f'G1 {a}0')
            states, _ = self.run_lines([f'G21 G90 G94 {plane} G91.1',
                                       f'G1 {a}5 F600', f'G2 {a}0 {b}5 {i}-5'])
            self.assertTrue(all(s[0] == 0 and s[1] != 2 for s in states.values()), states)

    def test_zero_centres_resolve_with_inches_and_work_offsets(self):
        for units in ('G20', 'G21'):
            for mode in ('G90.1', 'G91.1'):
                for shifted in (False, True):
                    setup = [f'{units} G90 G94 G17 {mode}', 'G1 X1 Y2 F60']
                    if shifted:
                        setup.append('G92 X3 Y4')
                    centre = ('I3 J4' if shifted else 'I1 J2') if mode == 'G90.1' else 'I0 J0'
                    with self.subTest(units=units, mode=mode, shifted=shifted):
                        self.check_refusal(setup, f'G2 X2 Y3 {centre}', 'G1 X0 Y0')


if __name__ == '__main__':
    unittest.main()
