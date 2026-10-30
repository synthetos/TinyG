"""E095: the CNCjs connect sequence puts edge into ALARM and its status-report setup never takes effect.

CNCjs's 246-char `{sr:{...}}` line overflows the 200-byte line-mode buffer (xio.h:215) and raises
`cm_alarm(STAT_BUFFER_FULL)` (controller.c:216-218); in char mode it hits the 128-char JSON cap
(json_parser.c:235-236). Even when it fits, the g2core-only keys spc/sps/com/cof/tof* make
_marshall reject the whole list (json_parser.c:401-402), so mpox never appears in status reports.
Master rejects the unknown keys too.
"""
import time

import pytest

from tinyg_hil import snapshot

SR_LINE = ("{sr:{stat:t,line:t,vel:t,feed:t,unit:t,coor:t,momo:t,plan:t,path:t,dist:t,admo:t,frmo:t,"
           "tool:t,posx:t,posy:t,posz:t,posa:t,posb:t,posc:t,mpox:t,mpoy:t,mpoz:t,mpoa:t,mpob:t,mpoc:t,"
           "spc:t,sps:t,com:t,cof:t,tofx:t,tofy:t,tofz:t,tofa:t,tofb:t,tofc:t}}")
# TinyGController.js:937-995, in order
CONNECT = ["{ej:1}", "{jv:4}", "{qv:1}", "{sv:1}", "{si:200}",
           "{spe:n}", "{spd:n}", "{spc:n}", "{sps:n}", "{com:n}", "{cof:n}",
           SR_LINE, "{sys:n}", "{mt:n}", "{pwr:n}", "{qr:n}", "{sr:n}"]


def _replay(tg, lines, timeout: float = 3.0):
    """Send each line after the previous response (or a timeout), like a one-in-flight host.

    Returns [(line, response event or None, every event seen for that line)].
    """
    out = []
    for line in lines:
        tg.transport.write(line.encode() + b"\n")
        events, resp, end = [], None, time.monotonic() + timeout
        while resp is None:
            ev = tg.transport.next_event(end - time.monotonic())
            if ev is None:
                break
            events.append(ev)
            if ev.kind == "response":
                resp = ev
        events += tg.drain(quiet=0.1)
        out.append((line, resp, events))
    return out


@pytest.mark.persists
@pytest.mark.bug("E095", on=["d440", "d449"])
def test_cncjs_connect_sequence(tg, keep):
    keep("ej", "jv", "qv", "sv", "si")
    original_sr = list(tg.get("sr"))
    assert len(SR_LINE) == 246
    try:
        results = _replay(tg, CONNECT)
    finally:
        tg.drain(quiet=0.2)
        tg.clear_alarm()
        if list(tg.get("sr")) != original_sr:
            snapshot.set_status_report(tg, original_sr)
    exceptions = [e.raw for _, _, events in results for e in events if e.kind == "exception"]
    assert not exceptions
    _, sr_resp, _ = results[CONNECT.index(SR_LINE)]
    assert sr_resp is not None and sr_resp.data["f"][1] == 0, sr_resp and sr_resp.raw
    _, final, _ = results[-1]
    sr = final.data["r"]["sr"]
    assert sr["stat"] != tg.dialect.stat("ALARM")
    assert "mpox" in sr
