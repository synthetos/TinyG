"""Diagnostic: map switchbox channels to this board's switch inputs (bench bring-up).

Every switch input is set to limit mode (normally open), the board is reset so the modes take effect, and each
switchbox channel is pulsed closed for 150 ms in turn. The board raises "Limit switch hit - input N" for the input
the channel drives (N = 1 X min, 2 X max, 3 Y min, 4 Y max, 5 Z min, 6 Z max, 7 A min, 8 A max); a channel that
raises nothing is not wired to this board. The alarm is cleared after each pulse. Settings are restored afterwards.
Needs TINYG_SWITCHBOX_PORT (the switchbox serial port). Records only.
"""
import os
import re
import time

import pytest

from tinyg_hil.switchbox import SwitchBox

pytestmark = [pytest.mark.diagnostic, pytest.mark.persists, pytest.mark.resets_board, pytest.mark.dialect("d449"),
              pytest.mark.timeout(900)]

INPUTS = {1: "xmin", 2: "xmax", 3: "ymin", 4: "ymax", 5: "zmin", 6: "zmax", 7: "amin", 8: "amax"}
MODES = [f"{a}{e}" for a in "xyza" for e in ("sn", "sx")]
LIMIT = 2


def test_switchbox_map(tg, keep, record):
    port = os.environ.get("TINYG_SWITCHBOX_PORT")
    if not port:
        pytest.skip("TINYG_SWITCHBOX_PORT not set")
    keep("st", *MODES)
    if tg.get("st") != 0:
        tg.set("st", 0, allow="normally open inputs for the map")
    for tok in MODES:
        tg.set(tok, LIMIT, allow="limit mode for the switchbox map")
    tg.reset()                                     # switch modes and type take effect from a reset
    if tg.stat() == tg.dialect.stat("ALARM"):
        tg.clear_alarm()
    sb = SwitchBox(port, lines="asserted", allow_reset=True, purpose="switchbox channel map", lock_wait=30).connect()
    found, levels = {}, {}
    try:
        sb.release_all()
        time.sleep(0.5)
        state = sb.state()
        names = [c["name"] for c in state["channels"]]
        levels["open"] = {c["name"]: c["level"] for c in state["channels"]}
        record("switchbox_info", {k: sb.info_at_connect.get(k) for k in ("fw_version", "board", "channels")})
        only = [n for n in os.environ.get("SWITCHMAP_CHANNELS", "").split(",") if n]
        pulse_ms = int(os.environ.get("SWITCHMAP_PULSE_MS", "150"))
        for name in (only or names):
            if tg.stat() == tg.dialect.stat("ALARM"):
                tg.clear_alarm()
            tg.drain(quiet=0.2)
            mark = len(tg.transport.history)
            sb.pulse_channel(name, ms=pulse_ms)
            time.sleep(0.45 + pulse_ms / 1000)
            try:
                tg.send_line('{"stat":null}', timeout=2.0)
            except Exception:  # noqa: BLE001 - the alarm reply may carry an error status
                pass
            exc = [e.raw for e in tg.transport.history[mark:] if e.kind == "exception"]
            hit = [int(m.group(1)) for x in exc for m in [re.search(r"input (\d+)", x)] if m]
            found[name] = {"inputs": sorted(set(hit)), "tinyg": [INPUTS.get(n, n) for n in sorted(set(hit))],
                           "exceptions": [x[:120] for x in exc[:2]]}
        sb.release_all()
    finally:
        sb.disconnect()
        if tg.stat() == tg.dialect.stat("ALARM"):
            tg.clear_alarm()
    wired = {n: v["tinyg"] for n, v in found.items() if v["inputs"]}
    record("switchbox_map", {"wired": wired, "all": found, "levels_open": levels.get("open")})
    print("WIRED:", wired)
    assert tg.get("fb") is not None
