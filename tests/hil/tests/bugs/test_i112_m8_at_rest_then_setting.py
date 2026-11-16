"""GitHub #112: a lone M-code at rest must not leave the machine 'machining' so that later settings are refused.

Report (2015): after `M8` with nothing else queued, the machine state stays MACHINING and a following `$` setting
fails ("file not open"). On edge a command block that empties the planner ends the cycle (planner.c
mp_runtime_command -> cm_cycle_end), and settings refused while work runs return status 117 (N60), so the refusal
must not happen at rest. The test turns coolant on and off at rest, waits for the planner to drain, and then
expects an idle machine state and an accepted persisted setting (written back to its own value).
"""
import pytest

pytestmark = [pytest.mark.persists, pytest.mark.dialect("d449"), pytest.mark.timeout(60)]


def test_setting_accepted_after_lone_m8_at_rest(tg, keep, record):
    keep("xjm")
    result = {}
    for code in ("M8", "M9", "M7", "M9"):
        tg.send_gcode(code, check=True, allow="#112 coolant at rest")
        tg.wait_idle(timeout=10)
        result[code + "_stat"] = tg.stat()
        reply = tg.send_json({"xjm": tg.get("xjm")})
        result[code + "_set_status"] = reply.status
    record("i112", result)
    for code in ("M8", "M9", "M7"):
        assert result[code + "_stat"] in tg.dialect.idle_stats, result
        assert result[code + "_set_status"] == 0, result
