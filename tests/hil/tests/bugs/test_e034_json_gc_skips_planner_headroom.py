"""E034: JSON-wrapped G-code is dispatched as a control line, ahead of the planner headroom check,
so a few {"txt":"<gcode>"} lines sent while the planner is full panic the board. ({"gc":...} would
take the same path, but edge currently discards it, E103, so the test uses txt.)

xio.c:541-542 marks every line starting with '{' DEV_IS_CTRL; controller.c:157 _dispatch_control()
runs above controller.c:175 _sync_to_planner(); gc_run_gc (config_app.c:499 -> gcode_parser.c:569)
has no headroom check, so mp_get_write_buffer() fails (planner.c:403, st 28) and plan_line.c:150 /
planner.c:217,260 call cm_panic (st 14). Master reads one line at a time behind _sync_to_planner.
"""
import time

import pytest

from tinyg_hil import NoResponse
from tinyg_hil.driver import encode

pytestmark = pytest.mark.dialect("d449")     # tid and PANIC are 449-only; 440 gates JSON lines behind the planner


def _fill_planner(tg):
    """Queue a long dwell, then short dwells until the planner stops accepting lines."""
    blocked_until = time.monotonic() + 5
    tg.send_gcode("G4 P5", check=True)
    for _ in range(40):
        try:
            tg.send_gcode("G4 P0.01", timeout=0.4)
        except NoResponse:
            return blocked_until  # this line waits in RX; its response comes later
    pytest.fail("planner never reached its headroom limit")


@pytest.mark.resets_board
@pytest.mark.persists
@pytest.mark.parametrize("rxm", [1, 0], ids=["line-mode", "char-mode"])
def test_json_gc_waits_for_planner_room(tg, keep, rxm):
    keep("rxm")
    tg.set("rxm", rxm, allow="test both receive modes")
    panic = tg.dialect.stat("PANIC")
    try:
        _fill_planner(tg)
        burst = "".join(encode({"txt": "G4 P0.01", "tid": k}) + "\n" for k in range(1, 7))
        tg.transport.write(burst.encode())      # one write, so all six arrive while the planner is full
        tids, bad, end = [], [], time.monotonic() + 20
        while len(tids) < 6:
            ev = tg.transport.next_event(end - time.monotonic())
            if ev is None:
                break
            if ev.kind == "exception" or (ev.kind == "status" and ev.body.get("stat") == panic):
                bad.append(ev.raw)
            elif ev.kind == "response" and ev.data.get("tid", ev.body.get("tid")) is not None:
                tids.append((ev.data.get("tid", ev.body.get("tid")), ev.data["f"][1]))
        assert not bad, bad
        assert tids == [(k, 0) for k in range(1, 7)]
        assert tg.stat() != panic
    finally:
        tg.reset()


@pytest.mark.resets_board
@pytest.mark.persists
def test_status_query_bypasses_full_planner(tg, keep):
    keep("rxm")
    tg.set("rxm", 1, allow="test control priority in line mode")
    try:
        blocked_until = _fill_planner(tg)
        assert time.monotonic() + 1.0 < blocked_until, "not enough dwell remaining to test priority"
        tg.transport.write((encode({"sr": None}) + "\n").encode())
        # A DATA line cannot answer until the five-second dwell releases planner room.
        ev = tg.wait_for(
            lambda ev: ev.kind == "response" and "sr" in ev.body,
            timeout=1.0,
        )
        assert ev.data["f"][1] == 0, ev.raw
        assert time.monotonic() < blocked_until, "status arrived after planner room became available"
    finally:
        tg.reset()
