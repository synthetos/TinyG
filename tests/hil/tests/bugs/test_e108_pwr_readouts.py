"""E108: the pwr1-pwr4 motor-power readouts were removed; {"pwr":null} returns an empty group.

config_app.c:173-184 comments out the pwr1..pwr6 rows while the `pwr` group row (:663) and
st_get_pwr (stepper.c:1246-1251) remain, so {"pwr1":null} gets 100 and {"pwr":null} gives {}.
Master has the rows (master config_app.c:175-178).
"""
import pytest


@pytest.mark.bug("E108", on=["d449"])
def test_pwr_group_has_motors(tg):
    body = tg.send_json({"pwr": None}, check=True).body["pwr"]
    keys = {k if k.startswith("pwr") else f"pwr{k}" for k in body}    # group members may be prefix-stripped
    assert {"pwr1", "pwr2", "pwr3", "pwr4"} <= keys, body
    assert all(v in (0, 1) for v in body.values()), body


@pytest.mark.bug("E108", on=["d449"])
def test_pwr_reads_off_after_md(tg):
    tg.send_json({"md": None}, check=True)
    assert [tg.get(f"pwr{m}") for m in range(1, 5)] == [0, 0, 0, 0]
