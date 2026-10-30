"""Diagnostic: follow a software-switch edge on X-min through the firmware (pin, port interrupt, sw.s[0])."""
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.ice]


def _snapshot(ice):
    base = ice.board.ports["A"]
    with ice.halted():
        regs = {"DIR": ice.read(base + 0x00, 1)[0], "IN": ice.read(base + 0x08, 1)[0],
                "INTCTRL": ice.read(base + 0x09, 1)[0], "INT0MASK": ice.read(base + 0x0A, 1)[0],
                "INTFLAGS": ice.read(base + 0x0C, 1)[0], "PIN6CTRL": ice.read(base + 0x16, 1)[0]}
        fw = ice.peek_many(["sw.s[0].state", "sw.s[0].mode", "sw.s[0].lockout_ms", "sw.s[0].timeout._start", "sw.s[0].timeout._delay", "rtc.sys_ticks"])
    return {"pin": (regs["IN"] >> 6) & 1, "regs": {k: f"0x{v:02x}" for k, v in regs.items()}, "fw": fw}


@pytest.mark.profile("switches-no")
def test_x_min_edge_reaches_firmware(tg, ice, record):
    ice.release_all_switches()
    time.sleep(0.2)
    record("released", _snapshot(ice))
    record("st", tg.get("st"))
    record("types", {"sw.switch_type": ice.peek("sw.switch_type"), "sw.s[0] raw": ice.peek("sw.s[0]").hex()})
    ice.set_switch_level("x", "min", 0)
    time.sleep(0.2)
    record("driven_low", _snapshot(ice))
    ice.set_switch_level("x", "min", 1)
    time.sleep(0.2)
    record("released_again", _snapshot(ice))
    ice.release_all_switches()


@pytest.mark.profile("switches-no")
def test_x_min_interrupt_flag_during_halt(tg, ice, record):
    """Does the port's edge detector flag a pin change made while the CPU is halted?"""
    base = ice.board.ports["A"]
    ice.release_all_switches()
    time.sleep(0.2)
    with ice.halted():
        before = ice.read(base + 0x0C, 1)[0]
        ice.write(base + 0x06, bytes([0x40]))          # OUTCLR bit 6
        ice.write(base + 0x01, bytes([0x40]))          # DIRSET bit 6
        in_halt = ice.read(base + 0x0C, 1)[0]
        pin_in_halt = (ice.read(base + 0x08, 1)[0] >> 6) & 1
    time.sleep(0.1)
    after = {"INTFLAGS": ice.read(base + 0x0C, 1)[0], "state": ice.peek("sw.s[0].state"),
             "timeout_start": ice.peek("sw.s[0].timeout._start")}
    record("flags", {"before": before, "in_halt": in_halt, "pin_in_halt": pin_in_halt, **after})
    ice.release_all_switches()
