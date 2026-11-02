"""E095: CNCjs capability probes and its full cross-controller SR setup must connect.

The full 246-character selector deliberately includes unsupported g2core fields. Only
boolean selections inside sr may ignore missing fields; unknown commands still fail.
The six capability probes (spe, spd, spc, sps, com, cof) are g2core's spindle and coolant readouts:
edge-next answers them with values, 440.21 with null (status 100, unrecognized).
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
@pytest.mark.bug("E095", on=["d440"])
def test_cncjs_connect_sequence(tg, keep):
    keep("ej", "jv", "qv", "sv", "si")
    original_sr = list(tg.get("sr"))
    assert len(SR_LINE) == 246
    try:
        results = _replay(tg, CONNECT)
    finally:
        tg.drain(quiet=0.2)
        tg.clear_alarm()
        snapshot.set_status_report(tg, original_sr)  # restore even if a large SR query failed
    exceptions = [e.raw for _, _, events in results for e in events if e.kind == "exception"]
    assert not exceptions
    for line, response, _ in results[5:11]:
        key = line[1:line.index(":")]
        assert response is not None, line
        if tg.dialect.name == "d449":           # g2core-named readouts, supported
            assert response.data["f"][1] == 0 and isinstance(response.data["r"].get(key), (int, float)), (line, response.raw)
        else:
            assert response.data["r"].get(key, "missing") is None, (line, response)
            assert response.data["f"][1] == 100  # unsupported capability, not successful execution
    _, sr_resp, _ = results[CONNECT.index(SR_LINE)]
    assert sr_resp is not None and sr_resp.data["f"][1] == 0, sr_resp and sr_resp.raw
    _, final, _ = results[-1]
    sr = final.data["r"]["sr"]
    assert sr["stat"] != tg.dialect.stat("ALARM")
    assert "mpox" in sr


@pytest.mark.dialect("d449")
@pytest.mark.parametrize("line", ['{zzzzq:1}', '{sys:{zzzzq:t}}', '{sr:{zzzzq:42}}', '{sr:{zzzzq:{stat:t}}}'])
def test_unknown_commands_still_fail(tg, line):
    assert tg.send_line(line).status == 100
