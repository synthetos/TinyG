"""N42: a homing cycle with no switch present cannot be ended by feedhold + queue flush on edge-next.

Sends G28.2 X0 on a board whose X switch never closes, then '!' and '%' as a host would, and records over the
ICE what the machine, cycle, hold, flush, runtime and stepper states are while it sits there, and whether a
{stat} query is answered. Then ^D, the same snapshot, then ^X to recover. Needs --ice and a hooks build (no
switch wired on the bench). Pure observation: the assertion only says whether the hold+flush ended the cycle.
"""
import os
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.ice, pytest.mark.resets_board,
              pytest.mark.dialect("d449"), pytest.mark.timeout(300)]

SYMBOLS = ["cm.machine_state", "cm.cycle_state", "cm.motion_state", "cm.hold_state", "cm.queue_flush_state",
           "cm.queue_flush_input", "cm.queue_flush_input_due", "cm.homing_state", "mr.block_state", "mr.section",
           "mr.segment_count", "st_pre.buffer_state", "st_pre.block_type", "st_run.dda_ticks_downcount",
           "mb.buffers_available", "cs.controller_state", "us[0].rx_buf_count", "us[0].fc_state_tx",
           "hm.axis", "hm.func", "hm.homing_state", "hm.open_count"]


def _snap(ice):
    out = {}
    try:
        vals = ice.peek_many(SYMBOLS)
        for k, v in zip(SYMBOLS, vals):
            out[k] = v
    except Exception:  # noqa: BLE001
        for k in SYMBOLS:
            try:
                out[k] = ice.peek(k)
            except Exception as e:  # noqa: BLE001
                out[k] = f"-({type(e).__name__})"
    try:
        out["where"] = ice.where()
    except Exception as e:  # noqa: BLE001
        out["where"] = f"-({type(e).__name__})"
    return out


def _answers(tg):
    """Any reply means the board is alive; its status is kept (a homing error rides on the next reply, 247)."""
    try:
        r = tg.send_line('{"stat":null}', timeout=2.0)
        return {"answered": True, "status": r.status, "raw": r.raw[:80]}
    except Exception as e:  # noqa: BLE001
        return {"answered": False, "raw": f"no reply: {type(e).__name__}"}


def test_homing_abort_state(tg, ice, keep, record):
    keep("xsn", "xsx", "xtm", "xsv", "xlv")
    tg.set("xsn", 1, allow="X-min as homing switch")
    tg.set("xsx", 0, allow="no X-max switch")
    tg.set("xtm", int(os.environ.get("HOMING_TRAVEL", "400")), allow="long search, so the hold lands mid-search")
    tg.send_gcode("G21 G90 G17 G94 G92.1 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0", check=True, allow="declare the position")
    tr = tg.transport
    info = {}
    tg.guard.check_gcode("G28.2 X0", allow="homing with no switch: the cycle under test")
    resp = tg.send_line("G28.2 X0", timeout=3.0)
    info["g28_2_status"] = resp.status
    time.sleep(1.0)
    info["during"] = {"query": _answers(tg), "ice": _snap(ice)}
    tr.write(b"!", drain_timeout=None); time.sleep(0.8)
    tr.write(b"%", drain_timeout=None); time.sleep(2.0)
    info["after_hold_flush"] = {"query": _answers(tg), "ice": _snap(ice),
                                "exceptions": [e.raw[:100] for e in tr.history[-40:] if e.kind == "exception"]}
    tr.write(b"\x04", drain_timeout=None); time.sleep(2.0)
    info["after_job_kill"] = {"query": _answers(tg), "ice": _snap(ice),
                              "exceptions": [e.raw[:100] for e in tr.history[-40:] if e.kind == "exception"]}
    record("homing_abort", info)
    try:
        tg.reset()
        tg.send_gcode("G21 G90 G54", allow="recover")
    except Exception:  # noqa: BLE001
        pass
    ended = info["after_hold_flush"]["query"]["answered"] and info["after_hold_flush"]["ice"].get("cm.cycle_state") in (0, 1)
    assert ended, f"hold+flush did not end the homing cycle: {info['after_hold_flush']}"
