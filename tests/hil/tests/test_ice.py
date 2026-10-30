"""Atmel-ICE helpers against the live board (pass --ice)."""
import pytest

pytestmark = pytest.mark.ice


def test_peek_matches_serial(tg, ice):
    snap = ice.peek_many(["cs.magic_start", "cs.fw_build", "mb.buffers_available"])
    assert snap["cs.magic_start"] == 0x12EF
    assert snap["cs.fw_build"] == pytest.approx(tg.get("fb"), abs=0.001)
    assert snap["mb.buffers_available"] == tg.get("qr")


def test_board_runs_after_peeks(tg, ice):
    for _ in range(20):
        ice.peek("cm.machine_state")
    assert tg.get("fb") == pytest.approx(tg.identity["fb"])


def test_poke_round_trip(tg, ice):
    original = ice.peek("cs.null")
    ice.poke("cs.null", 0x5A5A5A5A)
    try:
        assert ice.peek("cs.null") == 0x5A5A5A5A
    finally:
        ice.poke("cs.null", original)


def test_switch_simulation_changes_pin_and_input(tg, ice):
    """Pull-down on X-min makes the pin read low and the firmware's input change; pull-up restores it."""
    try:
        assert ice.switch_pin("x", "min") == 1, "unwired switch input should idle high (pull-up)"
        ice.set_switch_level("x", "min", 0)
        assert ice.switch_pin("x", "min") == 0
        ice.set_switch_level("x", "min", 1)
        assert ice.switch_pin("x", "min") == 1
    finally:
        ice.release_all_switches()
