"""Diagnostics for two source-review candidates (N16, N17). Needs --ice (the board may need a debugger reset).

N16 status-report recursion: {"sr":{"sr":true}} selects the report itself; the next {"sr":null} would print the list
    from inside printing the list. Records whether the set was refused, and whether the board survives the request.
N17 text-mode name prefix: $xvmzz=<value> in text mode: is it refused, or does it set xvm?
"""
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.ice, pytest.mark.persists, pytest.mark.resets_board,
              pytest.mark.timeout(300)]


def test_n16_sr_self_reference(tg, keep, record, ice):
    keep("sr")
    info = {}
    try:
        r = tg.send_line('{"sr":{"posx":true,"sr":true}}', timeout=3.0)
        info["set_status"] = r.status
        try:
            r2 = tg.send_line('{"sr":null}', timeout=3.0)
            info["sr_null_status"] = r2.status
            info["sr_null_reply"] = r2.raw[:160]
        except Exception as e:  # noqa: BLE001 - no reply: the board is gone
            info["sr_null"] = f"no reply: {type(e).__name__}"
        time.sleep(1.0)
        try:
            info["answers_after"] = tg.send_line('{"stat":null}', timeout=2.0).status == 0
        except Exception:  # noqa: BLE001
            info["answers_after"] = False
        if not info["answers_after"]:
            info["pc"] = sorted({ice.where() for _ in range(4)})
            info["state"] = ice.peek_many(["cm.machine_state", "cs.controller_state", "us[0].rx_buf_count"])
            ice.reset()
            time.sleep(3)
            tg.connect()
    finally:
        record("n16", info)
        try:
            tg.send_line('{"sr":{"posx":true,"posy":true,"posz":true,"vel":true,"stat":true}}')
        except Exception:  # noqa: BLE001
            tg.reset()
    assert info.get("set_status") != 0, f"the report accepted itself as a member: {info}"
    assert info.get("answers_after"), f"the board stopped answering after the self-referencing report: {info}"


def test_n17_text_name_prefix(tg, keep, record):
    keep("xvm")
    before = tg.get("xvm")
    info = {"before": before}
    probe = round(before * 0.5, 3) if before > 2 else before + 1
    lines = tg.send_text(f"$xvmzz={probe}")
    info["reply"] = [l[:80] for l in lines][:3]
    time.sleep(0.2)
    info["after"] = tg.get("xvm")
    record("n17", info)
    assert info["after"] == before, f"$xvmzz changed xvm: {info}"
