"""N11: hold and job kill must stop motion before a TX-paused host sends XON.

Uses the confirmed blocked-output reproduction. ICE proves state transitions;
measured stop latency remains a separate instrument gate.
"""
import time
import pytest
pytestmark=[pytest.mark.motion,pytest.mark.ice,pytest.mark.persists,
 pytest.mark.resets_board,pytest.mark.dialect("d449"),pytest.mark.bug("N11",on=[])]
XOFF,XON=b"\x13",b"\x11"

def test_n11_hold_ignored_while_tx_blocked(tg, keep, record, ice):
    keep("ex", "rxm", "ej", "si", "sv")
    for tok, val in (("ej", 1), ("rxm", 1), ("ex", 1), ("si", 100), ("sv", 1)):
        tg.set(tok, val, allow="xon/xoff for the blocked-output test")
    tg.send_gcode("G21 G90 G17 G94 G92.1 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0", check=True, allow="declare the position")
    info = {}
    tr = tg.transport
    try:
        tg.send_line("G91 G1 X40 F300")                        # 8 s move
        time.sleep(0.5)
        tr.write(XOFF, drain_timeout=None)                       # host says stop talking
        time.sleep(0.2)
        for _ in range(12):                                     # fill the board's TX ring behind the XOFF
            tr.write(b'{"x":null}\n', drain_timeout=None)
            time.sleep(0.05)
        time.sleep(0.5)
        tr.write(b"!", drain_timeout=None)
        time.sleep(1.5)
        info["after_hold_request"] = ice.peek_many(["cm.hold_state", "cm.motion_state", "mr.segment_velocity",
                                                     "us[0].fc_state_tx", "us[0].tx_buf_head", "us[0].tx_buf_tail"])
        info["pc"] = sorted({ice.where() for _ in range(4)})
        tr.write(b"\x04", drain_timeout=None)                    # job kill: also only honoured by the main loop
        time.sleep(1.0)
        info["after_job_kill"] = ice.peek_many(["cm.machine_state", "cm.hold_state", "cm.motion_state", "mr.segment_velocity"])
        tr.write(XON, drain_timeout=None)                        # host lets the board talk again
        time.sleep(1.5)
        info["after_xon"] = ice.peek_many(["cm.hold_state", "cm.motion_state", "mr.segment_velocity"])
    finally:
        tr.write(XON, drain_timeout=None)
        time.sleep(0.3)
        tg.reset()
        tg.send_gcode("G21 G90 G54", allow="recover")
    record("n11", info)
    held_before_xon = info["after_hold_request"].get("cm.motion_state") in (0, 2) or info["after_hold_request"].get("cm.hold_state", 0) >= 6
    assert info["after_job_kill"].get("cm.machine_state") == 2, f"job kill not alarmed: {info}"
    assert held_before_xon, f"'!' had no effect while output was blocked (hold waited for XON): {info}"

