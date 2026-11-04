"""N35 (reframed): after a genuine RX ring overflow, which later intact lines are reported "input lost"?

Overflows the ring on purpose the way the hang probe did by accident: starts the built-in self-test ({"test":1},
the board reads from flash and host input only queues) and sends OVERFLOW_BYTES of {stat} queries. When the test
ends and the overflow alarm is reported, the alarm is cleared. Then, one at a time, STEPS short intact lines
(GET, persisted SET, 0.5 mm move, as the probe's losing cases were) are sent and every reply is recorded; before each line the USB RX
bookkeeping is read over the ICE: us[0].rx_lost, rx_lost_open, rx_lost_first, rx_lost_last, rx_gap_report,
rx_line_damaged, rx_buf_head/tail/count, ds[0].len, ds[0].flag_in_line. A reply with status 13, or an "input
lost" exception, on an intact line after the clear is the finding; the ICE values say which flag caused it.
Needs --ice (the halts are between lines, never while one is in flight).
"""
import json
import os
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.ice, pytest.mark.persists,
              pytest.mark.resets_board, pytest.mark.dialect("d449"), pytest.mark.timeout(600)]

OVERFLOW_BYTES = int(os.environ.get("RX_OVERFLOW_BYTES", "400"))
STEPS = int(os.environ.get("RX_STEPS", "30"))
RX = ["us[0].rx_lost", "us[0].rx_lost_open", "us[0].rx_lost_first", "us[0].rx_lost_last", "us[0].rx_gap_report",
      "us[0].rx_line_damaged", "us[0].rx_buf_head", "us[0].rx_buf_tail", "us[0].rx_buf_count", "ds[0].len",
      "ds[0].flag_in_line", "cm.machine_state", "xio.buf_state",
      # flush-mark and loss-origin bookkeeping (read if the build has them; a missing symbol records '-')
      "us[0].rx_flush_marked", "us[0].rx_flush_mark", "us[0].rx_lost_after_mark", "us[0].rx_lost_post_first",
      "us[0].rx_lost_hw", "us[0].rx_lost_sw", "xio.rx_lost_count"]


def _rx(ice):
    try:
        vals = ice.peek_many(RX)                        # one halt; returns a dict keyed by expression
        return dict(vals) if isinstance(vals, dict) else dict(zip(RX, vals))
    except Exception:  # noqa: BLE001
        out = {}
        for k in RX:
            try:
                out[k] = ice.peek(k)
            except Exception as e:  # noqa: BLE001
                out[k] = f"-({type(e).__name__})"
        return out


def test_rx_overflow_aftermath(tg, ice, keep, record):
    keep("ja")
    tg.send_gcode("G21 G90 G17 G94 G92.1 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0 A0", check=True, allow="declare the position")
    tr = tg.transport
    info = {"before": _rx(ice)}
    tg.guard.check_json({"test": 1}, allow="self-test from flash: the input source switch under test")
    mark = len(tr.history)
    tr.write(b'{"test":1}\n', drain_timeout=None)
    time.sleep(0.3)
    q = b'{"stat":null}\n'
    for _ in range(OVERFLOW_BYTES // len(q) + 1):        # more than the ring holds while the board reads from flash
        tr.write(q, drain_timeout=None)
    end = time.monotonic() + 40                         # the self-test runs for a while; wait for the alarm
    alarmed = False
    while time.monotonic() < end:
        ev = tr.next_event(0.2)
        if ev is not None and ev.kind == "exception" and "input lost" in ev.raw:
            alarmed = True
        if alarmed:
            try:                                        # the overflow is charged to whatever line meets it,
                r = tg.send_line('{"stat":null}', timeout=2.0)   # possibly this query: status 13, no stat
                if isinstance(r.body, dict) and r.body.get("stat") == tg.dialect.stat("ALARM"):
                    break
            except Exception:  # noqa: BLE001
                pass
    info["overflow"] = {"alarmed": alarmed, "exceptions": [e.raw[:100] for e in tr.history[mark:] if e.kind == "exception"][:6],
                        "rx_after_overflow": _rx(ice)}
    while tr.next_event(0.2) is not None:               # drain the backlog of answers
        pass
    for _ in range(3):                                  # a query right after the overflow may itself get 13
        try:
            if tg.stat() == tg.dialect.stat("ALARM"):
                tg.clear_alarm()
            break
        except Exception:  # noqa: BLE001
            time.sleep(0.2)
    info["after_clear"] = _rx(ice)
    steps, lost = [], []
    for n in range(STEPS):
        # the probe's losing lines were a move right after an accepted persisted SET: GET, SET (real EEPROM
        # write, values alternate), move, repeated
        kind = n % 3
        line = ('{"ja":null}' if kind == 0 else
                ('{"ja":100000}' if (n // 3) % 2 == 0 else '{"ja":100001}') if kind == 1 else
                ("G91 G1 X0.5 F500" if (n // 3) % 2 == 0 else "G91 G1 X-0.5 F500"))
        if line.startswith("G"):
            tg.guard.check_gcode(line, allow="half a millimetre")
            tg.wait_idle(timeout=20)
        rx = _rx(ice)
        m = len(tr.history)
        try:
            r = tg.send_line(line, timeout=3.0)
            status, raw = r.status, r.raw[:80]
        except Exception as e:  # noqa: BLE001
            status, raw = f"no reply: {type(e).__name__}", ""
        time.sleep(0.2)
        exc = [e.raw[:100] for e in tr.history[m:] if e.kind == "exception"]
        step = {"n": n, "line": line, "status": status, "raw": raw, "exceptions": exc, "rx_before": rx}
        steps.append(step)
        if status == 13 or any("input lost" in x for x in exc):
            lost.append(step)
            try:
                if tg.stat() == tg.dialect.stat("ALARM"):
                    tg.clear_alarm()
            except Exception:  # noqa: BLE001
                pass
    info["steps"] = steps
    info["lost"] = lost
    record("rx_overflow_aftermath", info)
    try:
        tg.reset()
        tg.send_gcode("G21 G90 G54", allow="recover")
    except Exception:  # noqa: BLE001
        pass
    assert info["overflow"]["alarmed"], f"the ring did not overflow: {info['overflow']}"
    assert not lost, "intact lines reported lost after the overflow was cleared: " + json.dumps(
        [(s["n"], s["line"], s["status"], s["rx_before"]) for s in lost][:4])
