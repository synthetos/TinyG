"""Release images reject canned commands without starting or queuing motion."""
import pytest

pytestmark = pytest.mark.dialect("d449")


def test_disabled_canned_commands_leave_machine_idle(tg):
    probe = tg.send_json({"test": 99})
    if probe.status != 17:
        assert probe.status == tg.dialect.status("ERROR"), probe.raw
        pytest.skip("development image includes canned programs")
    help_lines = tg.send_text("$test")
    assert any("Canned self-tests are disabled" in line for line in help_lines), help_lines
    before = tg.get("qr")
    for command in ({"test": 1}, {"test": 51}, {"test": 99}):
        response = tg.send_json(command, allow="verify disabled canned command rejection")
        assert response.status == 17, response.raw
        assert tg.get("qr") == before
        assert tg.stat() in tg.dialect.idle_stats
    lines = tg.send_text("$test=4")
    assert any("err:" in line and "Function is stubbed" in line for line in lines), lines
    assert tg.get("qr") == before
    assert tg.stat() in tg.dialect.idle_stats
    assert tg.send_json({"test": 0}, check=True).status == 0
