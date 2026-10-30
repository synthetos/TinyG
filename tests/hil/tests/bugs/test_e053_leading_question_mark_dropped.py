"""E053: a line that starts with '?' followed by more text is silently dropped, with no response.

controller.c:279-282 (edge) `else if ((*cs.bufp == '?') && (strlen(cs.bufp) > 1)) { cs.bufp++; }`
is a link in _dispatch_kernel's else-if chain, so after the '?' is skipped no parser runs and the
host loses the response for that line. 440 sends such lines to the text parser instead.
"""
import time
import warnings

import pytest

from tinyg_hil import NoResponse

pytestmark = pytest.mark.dialect("d449")


@pytest.mark.bug("E053", on=[])
def test_leading_question_mark_line_gets_response(tg):
    resp = tg.send_line('?{"fb":null}', timeout=1.5)
    assert resp.status == 0
    assert resp.body.get("fb") == pytest.approx(tg.identity["fb"])


def test_plain_line_gets_response(tg):
    assert tg.send_line('{"fb":null}', timeout=1.5).body.get("fb") == pytest.approx(tg.identity["fb"])


def _fill_planner(tg):
    """Queue a long dwell, then short dwells until the planner stops accepting lines (as in E034)."""
    blocked_until = time.monotonic() + 5
    tg.send_gcode("G4 P5", check=True)
    for _ in range(40):
        try:
            tg.send_gcode("G4 P0.01", timeout=0.4)
        except NoResponse:
            return blocked_until
    pytest.fail("planner never reached its headroom limit")


def _char_mode(tg):
    tg.set("rxm", 0, allow="test both receive modes")
    first = tg.send_json({"fb": None})                 # E079 can garble the first line after the switch
    assert tg.get("fb") == pytest.approx(tg.identity["fb"])
    tg.drain()
    if first.status != 0:
        warnings.warn(f"E079: first GET after rxm=0 was corrupted: {first.raw}", RuntimeWarning)


# '?' plus G-code is G-code: it must wait behind the planner headroom check, not run as a control
# line while the planner is full (which panics, as in E034).
@pytest.mark.resets_board
@pytest.mark.persists
@pytest.mark.bug("E053", on=[])
@pytest.mark.parametrize("rxm", [1, 0], ids=["line-mode", "char-mode"])
def test_question_prefixed_gcode_waits_for_planner_room(tg, keep, rxm):
    keep("rxm")
    panic = tg.dialect.stat("PANIC")
    try:
        if rxm == 0:
            _char_mode(tg)
        else:
            tg.set("rxm", 1, allow="test both receive modes")
        _fill_planner(tg)
        tg.transport.write(b"".join(b"?G4 P0.01\n" for _ in range(6)))
        statuses, bad, end = [], [], time.monotonic() + 20
        while len(statuses) < 6:
            ev = tg.transport.next_event(end - time.monotonic())
            if ev is None:
                break
            if ev.kind == "exception" or (ev.kind == "status" and ev.body.get("stat") == panic):
                bad.append(ev.raw)
            elif ev.kind == "response":
                statuses.append(ev.data["f"][1])
        assert not bad, bad
        assert statuses == [0] * 6
        assert tg.stat() != panic
    finally:
        tg.reset()


@pytest.mark.resets_board
@pytest.mark.persists
def test_lone_question_mark_answers_while_planner_full(tg, keep):
    keep("rxm")
    tg.set("rxm", 1, allow="test control priority in line mode")
    try:
        blocked_until = _fill_planner(tg)
        assert time.monotonic() + 1.0 < blocked_until, "not enough dwell remaining to test priority"
        tg.transport.write(b"?\n")
        ev = tg.wait_for(lambda e: e.kind == "status", 1.0)
        assert ev.t < blocked_until
    finally:
        tg.reset()
