"""E056: a filtered status-report request that finds nothing new is never cleared.

report.c:428-430 returns before `sr.status_report_request = SR_OFF` (:432) when the filtered
report is empty, which only clears itself in PROGRAM_STOP/END (stat is always reported there).
In READY the stale request is re-populated every loop (the CPU burn) and, because requests are
first-wins (:393-395), it swallows the next timed request: the report after a G-code block then
goes out at once instead of after the status interval. That throttle bypass is what this checks.
"""
import pytest

pytestmark = pytest.mark.dialect("d449")     # uses {shutd:n}; master clears before populating

SI_MS = 2000
# G-code that changes one status-report field without moving
CHANGES = {"dist": "G91", "coor": "G55", "plan": "G18", "unit": "G20"}


def _status(stat=None, key=None):
    def pred(e):
        if e.kind != "status":
            return False
        return (stat is None or e.body.get("stat") == stat) and (key is None or key in e.body)
    return pred


@pytest.mark.persists
@pytest.mark.bug("E056", on=["d449"])
def test_report_after_gcode_respects_interval_in_ready(tg, keep):
    fields = list(tg.get("sr"))
    key = next((k for k in CHANGES if k in fields), None)
    if "stat" not in fields or key is None:
        pytest.skip(f"status report list lacks stat or a modal field: {fields}")
    keep("si")
    tg.drain(quiet=tg.get("si") / 1000 + 0.3)  # let any timed report already queued go out
    tg.set("si", SI_MS)

    tg.send_json({"shutd": None}, check=True)
    tg.wait_for(_status(stat=tg.dialect.stat("SHUTDOWN")), 3)
    tg.send_json({"clear": None}, check=True)  # SHUTDOWN clears to READY
    last = tg.wait_for(_status(stat=tg.dialect.stat("READY")), 3)
    tg.send_json({"fb": None}, check=True)     # filtered request with nothing new: stays pending on 449
    tg.send_gcode(CHANGES[key], check=True)
    report = tg.wait_for(_status(key=key), SI_MS / 1000 + 2)
    assert report.t - last.t >= 0.75 * SI_MS / 1000
