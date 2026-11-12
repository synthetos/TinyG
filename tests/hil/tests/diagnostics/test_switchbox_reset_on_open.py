"""Diagnostic: does opening or closing the switchbox's serial port reset the ESP8266?

The CP2104's DTR/RTS drive the Feather's auto-reset transistors: RTS asserted alone pulls EN low
(reset), DTR asserted alone pulls GPIO0 low. Each strategy in tinyg_hil.switchbox.LINE_STRATEGIES
gets several open/close sessions, 2 s apart. A reset on open shows as an uptime shorter than the
time since the port opened (or a boot line after it). A reset on close shows as a new boot_id at the
next session whose own open did not reset. Line events on amin (GPIO0, pulled up on the Feather)
between sessions would show a DTR-alone glitch long enough for the 10 ms poll to catch.
"""
import os
import time

import pytest

from tinyg_hil.lock import BoardLock
from tinyg_hil.switchbox import LINE_STRATEGIES, SwitchBox, resolve_port

pytestmark = [pytest.mark.diagnostic, pytest.mark.switchbox]

SESSIONS = 3
GAP_S = 2.0


@pytest.mark.timeout(300)
def test_reset_on_open_and_close(request, record):
    config = request.config
    port = resolve_port(config.getoption("--switchbox-port"), config.getoption("--port"))
    if not port or not os.path.exists(port):
        pytest.skip("no switchbox port present")
    plan = [(s, i) for s in LINE_STRATEGIES for i in range(SESSIONS)] + [("release-after-open", "probe")]
    sessions = []
    with BoardLock(port).acquire("diagnostic switchbox reset-on-open", wait=config.getoption("--lock-wait")):
        for strategy, i in plan:
            sb = SwitchBox(port, lock=False, allow_reset=True, lines=strategy)
            sb.connect()
            try:
                info = sb.info_at_connect
                prev = sessions[-1] if sessions else None
                between = []
                if prev and info["boot_id"] == prev["boot_id"]:
                    between = [e for e in sb.events(since=prev["head"], all=True)
                               if e["event"] in ("line", "boot") and e.get("ch") in ("amin", None)]
                sessions.append({"strategy": strategy, "session": i, "boot_id": info["boot_id"],
                                 "uptime_ms": info["uptime_ms"], "reset_on_open": sb.reset_on_connect,
                                 "boot_line_seen": sb.boot_seen, "garbage_lines": sb.garbage_count,
                                 "head": sb.head(), "amin_events_since_previous": between})
            finally:
                sb.disconnect()
            time.sleep(GAP_S)
    for prev, cur in zip(sessions, sessions[1:]):
        prev["reset_on_close"] = cur["boot_id"] != prev["boot_id"] and not cur["reset_on_open"]
    summary = {}
    for s in LINE_STRATEGIES:
        rows = [x for x in sessions if x["strategy"] == s and x["session"] != "probe"]
        summary[s] = {"opens": len(rows), "reset_on_open": sum(r["reset_on_open"] for r in rows),
                      "reset_on_close": sum(bool(r.get("reset_on_close")) for r in rows),
                      "amin_glitches_seen": sum(bool(r["amin_events_since_previous"]) for r in rows)}
    record("sessions", sessions)
    record("summary", summary)
    print(summary)
    assert any(v["reset_on_open"] == 0 and v["reset_on_close"] == 0 for v in summary.values()), summary
