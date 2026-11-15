"""Release images reject canned commands without starting or queuing motion."""
import pytest

pytestmark = pytest.mark.dialect("d449")


def test_disabled_canned_commands_leave_machine_idle(tg):
    help_response = tg.send_json({"txt": "$test"}, check=True)
    if not any("Canned self-tests are disabled" in line for line in help_response.text()):
        pytest.skip("development image includes canned programs")
    before = tg.get("qr")
    for command in ({"test": 1}, {"test": 51}, {"test": 99}, {"txt": "$test=4"}):
        response = tg.send_json(command, allow="verify disabled canned command rejection")
        assert response.status == 17, response.raw
        assert tg.get("qr") == before
        assert tg.stat() in tg.dialect.idle_stats
    assert tg.send_json({"test": 0}, check=True).status == 0
