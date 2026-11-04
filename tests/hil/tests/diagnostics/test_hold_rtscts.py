"""Diagnostic: can a host on hardware flow control (rxm=0 ex=2) resume from a feedhold once the board is full?

The host streams slow moves as fast as CTS allows until the planner is full and the board has dropped RTS,
sends '!' (which the RX interrupt handles even with a full ring), waits for HOLD, then sends '~'. With RTS/CTS the
FTDI holds every byte back while CTS is low, so '~' arrives only if the board lets more input in during the hold.
Records: bytes accepted before CTS dropped, whether HOLD was reached, whether '~' got through within RESUME_WAIT s,
and, when it did not, the CPU location and ring/planner state through the ICE before the ICE resets the board.
Same procedure on edge-0.97 and edge-next tells whether the deadlock is inherited or new.
"""
import os
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.resets_board, pytest.mark.timeout(600)]

RESUME_WAIT = 15.0
MAX_BYTES = 3000
PEEKS = ["cm.machine_state", "cm.hold_state", "mb.buffers_available", "us[0].rx_buf_count",
         "us[0].fc_state_rx", "us[0].fc_state_tx", "xio.rx_mode", "cs.controller_state"]


def _stat(tg, timeout=1.0):
    try:
        return tg.get("stat", timeout=timeout)
    except Exception:
        return None


def test_hold_rtscts(tg, keep, record, request):
    keep("rxm", "ex", "ej", "qv", "sv", "si")
    for tok, val in (("ej", 1), ("qv", 0), ("sv", 1), ("si", 250), ("rxm", 0), ("ex", 2)):
        try:
            tg.set(tok, val, allow="hardware flow control for the hold test")
        except Exception:                                    # edge-0.97's parser refuses {"ex":2}: text mode works
            tg.send_text(f"${tok}={val}")
            time.sleep(0.1)
    tg.send_gcode("G21 G90 G17 G94 G92.1 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0", check=True, allow="declare the position")
    ser = tg.transport.ser
    hold, info = tg.dialect.stat("HOLD"), {}
    lines = [f"G1 X{2 if k % 2 else 0} F100\n".encode() for k in range(400)]   # 2 mm at F100: 1.2 s each
    try:
        ser.rtscts = True
        ser.write_timeout = 1.0
        # 1. a modest burst: the planner fills (32 moves of 1.2 s), the RX ring still has room, CTS stays high
        sent = 0
        for line in lines[:40]:
            ser.write(line)
            sent += len(line)
        time.sleep(1.5)
        info["bytes_before_hold"], info["cts_before_hold"] = sent, ser.cts
        # 2. feedhold while the host still can reach the board
        tg.transport.write(b"!", drain_timeout=None)
        end = time.monotonic() + 10
        while time.monotonic() < end and _stat(tg) != hold:
            time.sleep(0.2)
        info["hold_reached"] = _stat(tg) == hold
        # 3. the host keeps streaming during the hold, as a sender that trusts CTS does, until CTS drops
        t0 = time.monotonic()
        for line in lines[40:]:
            try:
                ser.write(line)
                sent += len(line)
            except Exception:                                # write timed out: the host is blocked on CTS
                break
            if not ser.cts or sent >= MAX_BYTES or time.monotonic() - t0 > 20:
                break
            time.sleep(0.02)
        info["bytes_sent_total"], info["cts_in_hold"] = sent, ser.cts
        tg.transport.write(b"~", drain_timeout=None)
        end = time.monotonic() + RESUME_WAIT
        resumed = False
        while time.monotonic() < end:
            s = _stat(tg)
            if s is not None and s != hold:
                resumed = True
                break
            time.sleep(0.2)
        info["resumed"] = resumed
        info["cts_after_resume_attempt"] = ser.cts
        if not resumed and request.config.getoption("--ice", default=False):
            try:                                             # evidence first; a debugger hiccup must not hide it
                from tinyg_hil.ice import Ice, flashed_elf
                with Ice(elf=flashed_elf(tg.transport.port), port=tg.transport.port) as ice:
                    info["pc"] = sorted({ice.where() for _ in range(4)})
                    info["state"] = ice.peek_many(PEEKS)
                    ice.reset()
                time.sleep(3)
            except Exception as e:  # noqa: BLE001
                info["ice_error"] = f"{type(e).__name__}: {str(e)[:160]}"
    finally:
        record("hold_rtscts", info)                          # whatever was learned, even if recovery fails below
        ser.rtscts = False
        ser.write_timeout = 5
        try:
            tg.transport.write(b"\x18", drain_timeout=None)
            time.sleep(2.5)
            tg.connect()
        except Exception:
            tg.reset()
        tg.send_gcode("G21 G90 G54", allow="recover")
    assert info.get("hold_reached"), f"feedhold never reached HOLD: {info}"
    assert info.get("resumed"), f"'~' did not get through on hardware flow control: {info}"
