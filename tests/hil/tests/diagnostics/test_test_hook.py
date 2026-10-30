"""Diagnostic: is the firmware's test_hook mailbox serviced by the running main loop?"""
import collections
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.ice]


def test_hook_mailbox_is_serviced(tg, ice, record):
    if "test_hook" not in ice.symbols.sym:
        pytest.skip("firmware built without TEST_HOOKS")
    addr, size = ice.symbols.sym["test_hook"]
    addr -= ice.board.sram_symbol_base
    record("layout", {f: list(ice.symbols.resolve(f"test_hook.{f}")[:2]) for f in
                      ("magic", "pending", "axis", "bit", "action", "applied")})
    record("before", ice.read(addr, size).hex(" "))
    with ice.halted():
        ice.poke("test_hook.axis", 0)
        ice.poke("test_hook.bit", 6)
        ice.poke("test_hook.action", 0)              # release (harmless)
        ice.poke("test_hook.pending", 1)
        record("written", ice.read(addr, size).hex(" "))
    samples = []
    for _ in range(10):
        time.sleep(0.05)
        samples.append(ice.read(addr, size).hex(" "))
    record("after", samples)
    record("pc", collections.Counter(ice.where() for _ in range(15)).most_common(5))
