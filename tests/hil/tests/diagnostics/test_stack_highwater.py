"""Diagnostic: peak stack depth of the running firmware under a JSON/text/motion workload.

The free RAM between __bss_end and the stack pointer is painted through the Atmel-ICE, the workload
runs, then the lowest overwritten address gives the stack high-water mark. The worst case of any
code path not exercised here is not covered, so treat the margin as an upper bound on headroom.
"""
import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.ice]

WORKLOAD_JSON = ['{"sr":null}', '{"sys":null}', '{"x":null}', '{"y":null}', '{"z":null}', '{"a":null}',
                 '{"1":null}', '{"2":null}', '{"3":null}', '{"4":null}', '{"p1":null}', '{"g54":null}',
                 '{"qr":null}', '{"hom":null}', '{"prb":null}',
                 '{"gc":"G90 G21 G17 G1 X0.5 Y0.5 F600"}',
                 '{"gc":"G2 X0 Y0 I-0.25 J-0.25 F600"}', '{"gc":"G0 X0 Y0"}']


def test_stack_highwater(tg, ice, record):
    sp = ice.stack_pointer()
    start, end = ice.stack_paint()
    record("layout", {"bss_end": hex(start), "sp_at_paint": hex(sp), "painted_bytes": end - start})
    for line in WORKLOAD_JSON:
        tg.send_line(line, timeout=5)
    tg.wait_idle(timeout=30)
    for line in ("$$", "$x", "$sys"):                       # text-mode dumps (deep printf paths)
        tg.send_text(line, timeout=5)
    tg.send_line('{"gc":"G1 X0.2 Y0.1 Z0.0 A0 F300 (MSG a fairly long message riding on a motion line to stress the parser)"}',
                 timeout=5)
    tg.wait_idle(timeout=30)
    low, free = ice.stack_highwater(start, end)
    record("highwater", {"lowest_stack_address": hex(low), "min_free_bytes": free,
                         "max_stack_bytes": 0x5FFF - low + 1})
    assert free > 256, f"only {free} bytes between static data and the deepest stack use"
