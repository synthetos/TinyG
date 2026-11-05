"""N60: a persisted setting written while a cycle is active (moving or in feedhold) must be refused before it applies.

Stock edge (450.01, ab0b1a6f): nv_set() applied the value in RAM and only the EEPROM write failed, with status 106
"Can't save when moving" (persistence.c write_persistent_value). The board then ran on the new value until the next
reset and reverted after it; text mode even printed the new value and an ok prompt. Bench 2, 2026-11-04 21:22.

Required (maintainer decision 2026-11-04): while cm.cycle_state != CYCLE_OFF, a SET of any persisted setting, alone,
inside a group, or in text mode, is refused with status 117 "Setting refused: stop the job first" before anything
changes. RAM and EEPROM keep the old value, the job continues (no alarm, no hold), and GETs keep working. Idle SETs
work as before. The TinyG wiki 'TinyG Configuration' lists settings as configuration, not runtime controls; G10
offsets keep their own deferred write (Gcode-Support).
"""
import time

import pytest

pytestmark = [pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"), pytest.mark.timeout(300),
              pytest.mark.bug("N60", on=[])]

REFUSED = 117
MOVE = "G91 G1 X20 F100"          # 12 s at F100: every SET below lands mid-move


def _start_move(tg):
    tg.send_gcode("G21 G90", allow="units")
    tg.send_gcode("G28.3 X0", allow="declare X")
    tg.guard.check_gcode(MOVE, allow="N60 move")
    tg.send_line(MOVE)
    time.sleep(1.5)


def _finish(tg):
    tg.wait_idle(timeout=30)
    tg.send_gcode("G90", allow="absolute")


def test_set_in_cycle_is_refused_before_it_applies(tg, keep, record):
    keep("xvm", "xjm")
    vm, jm = tg.get("xvm"), tg.get("xjm")
    new_vm = vm - 100 if vm > 200 else vm + 100
    rows = {}

    _start_move(tg)
    moving = tg.dialect.stat("RUN")
    r = tg.send_json({"xvm": new_vm}, allow="N60: SET during motion")
    rows["json"] = {"status": r.status, "stat": tg.stat(), "xvm": tg.get("xvm")}
    r = tg.send_json('{"x":{"vm":%g,"jm":%g}}' % (new_vm, jm + 1), allow="N60: group SET during motion")
    rows["group"] = {"status": r.status, "stat": tg.stat(), "xvm": tg.get("xvm"), "xjm": tg.get("xjm")}
    text = tg.send_text("$xvm=%g" % new_vm)
    rows["text"] = {"lines": text, "stat": tg.stat(), "xvm": tg.get("xvm")}
    _finish(tg)
    rows["after_idle"] = {"xvm": tg.get("xvm"), "xjm": tg.get("xjm")}

    _start_move(tg)
    tg.send_raw(b"!")
    tg.wait_for(lambda e: e.kind == "status" and e.data.get("sr", {}).get("stat") == tg.dialect.stat("HOLD"),
                timeout=10)
    r = tg.send_json({"xvm": new_vm}, allow="N60: SET in feedhold")
    rows["hold"] = {"status": r.status, "stat": tg.stat(), "xvm": tg.get("xvm")}
    tg.send_raw(b"~")
    _finish(tg)

    r = tg.send_json({"xvm": new_vm}, allow="N60: idle SET still works")
    rows["idle"] = {"status": r.status, "xvm": tg.get("xvm")}
    record("n60", {"xvm": vm, "xjm": jm, "written": new_vm, "rows": rows})

    for case in ("json", "group", "hold"):
        assert rows[case]["status"] == REFUSED, f"{case}: {rows[case]}"
    for case in ("json", "group", "text", "hold"):
        assert rows[case]["xvm"] == vm, f"{case} changed the live value: {rows[case]}"
    assert rows["group"]["xjm"] == jm, f"group SET applied a child: {rows['group']}"
    assert any(str(REFUSED) in ln or "stop the job first" in ln for ln in text), f"text mode did not report: {text}"
    for case in ("json", "group", "text"):
        assert rows[case]["stat"] == moving, f"{case}: the job did not keep running: {rows[case]}"
    assert rows["after_idle"] == {"xvm": vm, "xjm": jm}, rows["after_idle"]
    assert rows["idle"]["status"] == 0 and rows["idle"]["xvm"] == new_vm, f"idle SET broken: {rows['idle']}"
