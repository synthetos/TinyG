"""E059: persisted spindle enable/direction polarity and static output levels.

Default polarity remains active-high enable and CW-low direction. ICE checks
verify register state only; startup glitches need external capture.
"""
import pytest


@pytest.mark.bug("E059", on=["d440"])
def test_polarity_tokens(tg):
    assert tg.get("spep") in (0, 1)
    assert tg.get("spdp") in (0, 1)


# Only the direction polarity is flipped: inverting spep would energize an active-high spindle output.
@pytest.mark.persists
@pytest.mark.resets_board
@pytest.mark.bug("E059", on=["d440"])
def test_dir_polarity_persists(tg, keep):
    keep("spdp")
    flipped = 1 - tg.get("spdp")
    tg.set("spdp", flipped)
    tg.reset()
    assert tg.get("spdp") == flipped


@pytest.mark.parametrize("token", ["spep", "spdp"])
@pytest.mark.parametrize("value", [2, 255])
@pytest.mark.bug("E059", on=["d440"])
def test_polarity_rejects_invalid_values(tg, token, value):
    before = tg.get(token)
    response = tg.send_json({token: value})
    assert response.status == tg.dialect.status("INPUT_VALUE_RANGE_ERROR")
    assert tg.get(token) == before


@pytest.mark.bug("E059", on=["d440"])
def test_polarity_cannot_change_during_queued_work(tg):
    """Reject even an unchanged value while a dwell occupies the runtime."""
    before = tg.get("spdp")
    try:
        tg.send_gcode("G4 P2", check=True)
        response = tg.send_json({"spdp": before})
        assert response.status == tg.dialect.status("COMMAND_NOT_ACCEPTED")
        assert tg.get("spdp") == before
    finally:
        tg.wait_idle(timeout=5)


@pytest.mark.ice
@pytest.mark.persists
@pytest.mark.resets_board
@pytest.mark.parametrize("enable_polarity", [0, 1])
@pytest.mark.parametrize("direction_polarity", [0, 1])
@pytest.mark.bug("E059", on=["d440"])
def test_polarity_output_latches(tg, ice, keep, enable_polarity, direction_polarity):
    """Bare-board static register check; does not establish glitch-free startup timing."""
    import time

    if not tg.guard.allow_motion:
        pytest.skip("requires --allow-motion on a bare board: changes spindle output levels")
    keep("spep", "spdp")

    def command(line, expected):
        tg.send_gcode(line, check=True)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if ice.peek("cm.gm.spindle_mode") == expected:
                return
            time.sleep(0.02)
        pytest.fail(f"spindle command did not execute: {line}")

    try:
        command("M5", 0)
        tg.wait_idle()
        tg.set("spep", enable_polarity)
        tg.set("spdp", direction_polarity)
        tg.reset()
        assert tg.get("spep") == enable_polarity
        assert tg.get("spdp") == direction_polarity
        # ATxmega192A3 PORTA/PF GPIO1: DIR at +0, OUT at +4, bit 5.
        assert ice.read(0x600, 1)[0] & 0x20
        assert bool(ice.read(0x604, 1)[0] & 0x20) == (not enable_polarity)
        for line, mode, direction_high in (("M3", 1, direction_polarity),
                                           ("M4", 2, not direction_polarity)):
            command(line, mode)
            assert bool(ice.read(0x604, 1)[0] & 0x20) == bool(enable_polarity)
            assert bool(ice.read(0x6A4, 1)[0] & 0x20) == bool(direction_high)
            response = tg.send_json({"spdp": direction_polarity})
            assert response.status == tg.dialect.status("COMMAND_NOT_ACCEPTED")
        command("M5", 0)
        assert bool(ice.read(0x604, 1)[0] & 0x20) == (not enable_polarity)
    finally:
        command("M5", 0)
        tg.wait_idle()
