"""N102 must return its session-scoped debugger open, including failed boot waits."""
import ast
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace
import unittest

SOURCE = Path(__file__).parents[1] / 'hil/tests/bugs/test_n102_rts_hold.py'


def helper(name):
    tree = ast.parse(SOURCE.read_text())
    nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name]
    if not nodes:
        raise AssertionError(f'missing helper {name}')
    ns = {}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), 'exec'), ns)
    return ns[name]


class Recovery(unittest.TestCase):
    def test_shared_debugger_reopened_after_success_and_failure(self):
        for failure in (None, 'reset', 'banner', 'query'):
            with self.subTest(failure=failure):
                events = []

                class Debugger:
                    opened = True
                    def reset(self):
                        assert self.opened
                        events.append('reset')
                        if failure == 'reset': raise RuntimeError('reset failed')
                    def close(self):
                        self.opened = False
                        events.append('close')
                    def open(self):
                        assert not self.opened
                        self.opened = True
                        events.append('open')

                ice = Debugger()

                class Board:
                    def drain(self, **kwargs): pass
                    def wait_for(self, predicate, **kwargs):
                        assert not ice.opened  # target boots only after detach
                        assert predicate(SimpleNamespace(kind='banner', body={'msg': 'SYSTEM READY'}))
                        events.append('banner')
                        if failure == 'banner': raise RuntimeError('no banner')
                    def get(self, token, **kwargs):
                        assert token == 'rxm'
                        events.append('query')
                        if failure == 'query': raise RuntimeError('no answer')
                        return 0

                if failure:
                    with self.assertRaises(RuntimeError): helper('_reset_and_wait')(Board(), ice, lambda *args: None)
                else:
                    helper('_reset_and_wait')(Board(), ice, lambda *args: None)
                self.assertTrue(ice.opened, events)
                self.assertEqual(events[-1], 'open')

    def test_hold_sample_does_not_require_status_reports(self):
        class Debugger:
            state = 0
            @contextmanager
            def halted(self): yield
            def peek_many(self, names):
                assert names == ['cm.hold_state']
                return {'cm.hold_state': self.state}
        ice = Debugger()
        for state in range(7):
            ice.state = state
            self.assertEqual(helper('_hold_reached')(ice), state >= 4)


if __name__ == '__main__': unittest.main()
