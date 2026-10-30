"""E125: {"test":N} is rejected with 110 for every N>=2 because the test row is typed boolean.

config_app.c:477 `test,_b0 ... run_test`; json_parser.c:450-458 range-checks booleans to 0..1.
Master typed it _f0 (master config_app.c:199). Test 99 does not exist, so run_test only prints
"Test #99 not found" and returns STAT_ERROR (test.c:79-81); tests 1-14 and 50-51 run motion programs.
"""
import pytest


@pytest.mark.bug("E125", on=["d449"])
def test_test_number_reaches_run_test(tg):
    resp = tg.send_json({"test": 99}, allow="test 99 does not exist; run_test only prints 'not found'")
    assert resp.status == tg.dialect.status("ERROR")
    assert any("not found" in line for line in resp.text())
