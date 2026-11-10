"""N106: the latch test must receive the failure report before halting for its final snapshot."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

from tinyg_hil.protocol import parse_line


def test_latch_failure_waits_for_report_before_debugger_snapshot():
    path = Path(__file__).parent / 'bugs/test_n15_homing_release_check.py'
    spec = importlib.util.spec_from_file_location('n15_observation', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    order = []
    pending = {'report': False}

    class TG:
        dialect = SimpleNamespace(status=lambda name: 240)
        transport = SimpleNamespace(history=[])
        def set(self, *args, **kwargs): pass
        def send_gcode(self, *args, **kwargs): pass
        def drain(self): return []
        def reset(self): pending['report'] = False
        def wait_for(self, predicate, timeout):
            event = parse_line('{"er":{"st":240,"msg":"Homing error - X latch did not release"}}', 1.0)
            assert predicate(event)
            self.transport.history.append(event)
            pending['report'] = False
            order.append('report')
            return event

    class Ice:
        symbols = SimpleNamespace(sym={'test_hook': 1})
        def peek(self, name): return -.2
        def peek_many(self, names):
            assert not pending['report'], 'debugger halted while the homing report was still pending'
            order.append('snapshot')
            return {'cm.cycle_state': 0, 'cm.homed[0]': 0, 'sw.s[0].state': 1, 'mr.position[0]': .625}

    class Switches:
        def unmount_all(self): pass
        def open(self, *args): pass
        def close(self, *args): pending['report'] = True

    module.test_latch_must_release_switch(TG(), lambda *args: None, Switches(), Ice(), lambda *args: None)
    assert order == ['report', 'snapshot']
