"""N35 recipe: an accepted persisted SET, then the next line at once, the way the hang probe sent it.

The hang probe lost the line after {"ja":0} and {"si":99999999} three runs out of three on 450.01, while the
strict-xfail test for N35 (SET then G1, nothing in between) passed 60 pairs out of 60 on the same build. The
probe's sequence differs in what came just before: "G28.3 X0 Y0 Z0 A0" (a queued planner command) and a GET of
the same token. This runs the probe's exact sequence and two controls, REPEATS times each, for ja, si and xvm:
  A  G28.3, GET token, SET token, G1          (the probe)
  B  GET token, SET token, G1                 (no planner command before)
  C  G28.3, SET token, G1                     (no GET before)
  D  the probe's whole preamble: "M5 M9 G21 G90 G17 G94 G92.1 G54", "G91.1", G28.3, GET, SET, G1
  E  as D, with the probe's values (ja 0, si 99999999, xvm 1): accepted on 450.01, refused by later fixes
Every case records the SET and G1 statuses, any exception the board reported and how far X moved: a lost G1
shows as "input lost" (edge-next), or as no reply / no motion (edge-0.97, which does not report lost input).
One lost line fails the test, so it reads: stock fails, fix passes. Values alternate so every SET writes.
"""
import json
import os
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"),
              pytest.mark.timeout(900)]

REPEATS = int(os.environ.get("PERSIST_REPEATS", "6"))
VALUES = {"ja": (100000, 100001), "si": (250, 251), "xvm": (1200, 1201)}
PROBE_VALUES = {"ja": (0, 100000), "si": (99999999, 250), "xvm": (1, 800)}
PREAMBLE = ("M5 M9 G21 G90 G17 G94 G92.1 G54", "G91.1")


def test_persist_then_line(tg, keep, record):
    keep("rxm", "ex", "ej", *VALUES)
    for tok, val in (("ej", 1), ("rxm", 1), ("ex", 1)):
        tg.set(tok, val, allow="line mode, as the probe ran")
    tg.send_gcode("G21 G90 G17 G94 G92.1 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0 A0", check=True, allow="declare the position")
    tr = tg.transport
    cases, lost = [], []
    for tok in VALUES:
        tg.wait_idle(timeout=30)
        tg.set(tok, VALUES[tok][1], allow="start from the other value")
        for recipe in "ABCDE":
            v0, v1 = (PROBE_VALUES if recipe == "E" else VALUES)[tok]
            for n in range(REPEATS):
                val = (v0, v1)[n % 2]
                move = "G91 G1 X0.5 Y0.5 F500" if n % 2 == 0 else "G91 G1 X-0.5 Y-0.5 F500"
                tg.guard.check_gcode(move, allow="half a millimetre, back and forth")
                tg.wait_idle(timeout=30)
                if tg.stat() == tg.dialect.stat("ALARM"):
                    tg.clear_alarm()
                x0 = tg.get("mpox")
                mark = len(tr.history)
                if recipe in "DE":
                    for line in PREAMBLE:
                        tg.send_gcode(line, allow="the probe's cleanup lines")
                if recipe in "ACDE":
                    tg.send_gcode("G28.3 X0 Y0 Z0 A0", allow="declare the position, as the probe did")
                if recipe in "ABDE":
                    tg.send_line(json.dumps({tok: None}), timeout=3.0)
                t0 = time.monotonic()
                set_status = tg.send_line(json.dumps({tok: val}), timeout=3.0).status
                t1 = time.monotonic()
                try:
                    status = tg.send_line(move, timeout=3.0).status
                except Exception as e:  # noqa: BLE001
                    status = f"no reply: {type(e).__name__}"
                time.sleep(0.3)
                try:
                    tg.wait_idle(timeout=30)
                except Exception:  # noqa: BLE001
                    pass
                if tg.stat() == tg.dialect.stat("ALARM"):
                    tg.clear_alarm()
                x1 = tg.get("mpox")                         # G28.3 declared X0 first in the recipes that send it
                moved = round(abs(x1) if recipe in "ACDE" else abs(x1 - x0), 3)
                exc = [e.raw[:100] for e in tr.history[mark:] if e.kind == "exception"]
                case = {"token": tok, "recipe": recipe, "n": n, "value": val, "set_status": set_status,
                        "set_reply_ms": round((t1 - t0) * 1000, 1), "move_status": status, "moved_mm": moved,
                        "exceptions": exc}
                cases.append(case)
                if any("input lost" in x for x in exc) or status == 13 or (set_status == 0 and status == 0 and moved < 0.4):
                    lost.append(case)
                if tok == "si":                              # the probe's value stops status reports; undo it
                    tg.send_line('{"si":250}', timeout=3.0)
    tg.wait_idle(timeout=30)
    if tg.stat() == tg.dialect.stat("ALARM"):
        tg.clear_alarm()
    tg.send_gcode("G90", check=True)
    by = {}
    for c in cases:
        k = f"{c['token']}-{c['recipe']}"
        by.setdefault(k, [0, 0])
        by[k][1] += 1
        by[k][0] += c in lost
    record("persist_then_line", {"lost_of_total": {k: f"{a}/{b}" for k, (a, b) in by.items()}, "lost": lost[:12],
                                 "cases": cases})
    assert not lost, "input lost after an accepted persisted SET: " + json.dumps({k: f"{a}/{b}" for k, (a, b) in by.items()})
