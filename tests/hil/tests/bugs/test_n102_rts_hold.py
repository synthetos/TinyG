"""A 192-byte unanswered RTS stream must leave real-time controls reachable during a hold.

Stock uses the early software XOFF watermark for RTS too. With a held/full planner the RX ring stops draining,
CTS stays off, and resume sits behind data at the host. A bounded sender must retain every queued byte while
keeping the wire open for resume. The ICE observes hold/RX/RTS state; teardown resets before settings restore.
"""
import time

import pytest
import serial

from tinyg_hil import gcodeplay

pytestmark = [pytest.mark.motion, pytest.mark.ice, pytest.mark.dialect("d449"),
              pytest.mark.resets_board, pytest.mark.timeout(240)]

PORTC = 0x0640                                           # DIR +0, OUT +4, IN +8; RTS is PC0
CAP = 192                                                # unanswered bytes, as the capped rts sender (N102 fix)


def board_side(ice):
    with ice.halted():
        pc = ice.read(PORTC, 9)
        vals = ice.peek_many(["cm.hold_state", "cm.motion_state", "cm.machine_state", "mb.buffers_available",
                              "us[0].rx_buf_count", "us[0].rx_lost", "us[0].fc_state_rx", "us[0].fc_state_tx", "xio.enable_flow_control"])
    vals["rts_out"] = pc[4] & 1
    vals["rts_pin"] = pc[8] & 1
    return vals


def host_side(ser):
    return {"cts": ser.cts, "out_waiting": ser.out_waiting}


def _reset_and_wait(tg, ice, record):
    # Purge pre-reset events, then send nothing through the bootloader's listen window.
    tg.drain(quiet=0.05, limit=0.5)
    try:
        try:
            ice.reset()
        finally:
            # Release PDI so the target boots before settings are restored.
            ice.close()
        tg.wait_for(lambda ev: ev.kind == "banner" and ev.body.get("msg") == "SYSTEM READY", timeout=20.0)
        value = tg.get("rxm", timeout=3.0)
        record("reset_ready", {"rxm": value})
    finally:
        # The fixture is session-scoped: even a failed boot wait must not leave later tests disconnected.
        ice.open()


def _hold_reached(ice):
    with ice.halted():
        return ice.peek_many(["cm.hold_state"])["cm.hold_state"] >= 4


@pytest.mark.bug("N102", on=[])
def test_rts_hold(tg, ice, keep, record):
    keep("rxm"); keep("ex"); keep("qv"); keep("ej")
    tg.set("ej", 1); tg.set("qv", 0)
    tg.send_gcode("G21 G90 G92 X0 Y0 Z0", check=True)
    tg.wait_idle(timeout=10)
    tg.set("rxm", 0); tg.set("ex", 2)
    lines = [l for l in gcodeplay.goose(passes=6) if l and not l.startswith("(")]
    for l in lines:
        tg.guard.check_gcode(l, allow="goose inside a 20 mm box from the G92 origin")
    ser = tg.transport.ser
    ser.rtscts = True
    saved_wt = ser.write_timeout
    ser.write_timeout = 0.05
    timeline = []
    i, t0, inflight = 0, time.monotonic(), []
    try:
        # stream for 3 s keeping at most CAP unanswered bytes (as the capped rts sender does), so '!' is never
        # queued behind more than about one RX ring of G-code; CTS still paces the tty
        while time.monotonic() - t0 < 3.0 and i < len(lines):
            ev = tg.transport.next_event(0.002)
            while ev is not None:
                if ev.kind == "response" and inflight:
                    inflight.pop(0)
                ev = tg.transport.next_event(0)
            data = (lines[i] + "\n").encode()
            if sum(inflight) + len(data) > CAP:
                continue
            try:
                ser.write(data)
                inflight.append(len(data))
                i += 1
            except serial.SerialTimeoutException:
                pytest.fail("bounded RTS write timed out; partial write not retried")
        sent_before_hold = i
        timeline.append(("before !", host_side(ser), {"unanswered_bytes": sum(inflight)}))
        ser.write_timeout = 5.0
        try:
            ser.write(b"!")
            bang_queued = True
        except serial.SerialTimeoutException:
            bang_queued = False
        t_bang = time.monotonic()
        held_at = None
        next_hold_poll = t_bang
        ser.write_timeout = 0.05
        # a host keeps streaming during a hold (it does not know when the board stops consuming): keep the capped
        # stream going for 6 s after '!', reading answers, and note when the board reports the hold
        while time.monotonic() - t_bang < 6.0:
            ev = tg.transport.next_event(0.002)
            while ev is not None:
                if ev.kind == "response" and inflight:
                    inflight.pop(0)
                ev = tg.transport.next_event(0)
            now = time.monotonic()
            if held_at is None and now >= next_hold_poll:
                if _hold_reached(ice):
                    held_at = time.monotonic() - t_bang
                next_hold_poll = time.monotonic() + 0.5
            if i < len(lines):
                data = (lines[i] + "\n").encode()
                if sum(inflight) + len(data) <= CAP:
                    try:
                        ser.write(data)
                        inflight.append(len(data))
                        i += 1
                    except serial.SerialTimeoutException:
                        pytest.fail("bounded RTS write timed out; partial write not retried")
        timeline.append(("streamed through the hold", host_side(ser), {"unanswered_bytes": sum(inflight), "lines_sent": i}))
        time.sleep(2.0)
        in_hold = (host_side(ser), board_side(ice))
        timeline.append(("in hold +2 s", *in_hold))
        ser.write_timeout = 2.0
        try:
            ser.write(b"~")
            tilde_queued = True
        except serial.SerialTimeoutException:
            tilde_queued = False
        time.sleep(3.0)
        after = (host_side(ser), board_side(ice))
        timeline.append(("after ~ +3 s", *after))
        record("n102", {"lines_sent_before_hold": sent_before_hold, "lines_total": len(lines),
                        "bang_queued": bang_queued, "hold_after_s": held_at, "tilde_queued": tilde_queued,
                        "timeline": timeline})
        h, b = in_hold
        deadlock = b["cm.hold_state"] >= 4 and b["rts_pin"] == 1 and after[1]["cm.hold_state"] >= 4
        record("verdict", {"board_rts_off_in_hold": b["rts_pin"] == 1, "hold_ended_after_tilde": after[1]["cm.hold_state"] == 0,
                           "deadlock": deadlock})
        print("N102", {"held_after_s": held_at, "in_hold": in_hold, "after_tilde": after})
        assert bang_queued and held_at is not None, timeline
        assert h["cts"] and b["rts_pin"] == 0, timeline
        assert tilde_queued and after[1]["cm.hold_state"] == 0, timeline
        assert not b["us[0].rx_lost"] and not after[1]["us[0].rx_lost"], timeline
        assert after[1]["cm.machine_state"] not in tg.dialect.stats("ALARM", "SHUTDOWN", "PANIC"), timeline
    finally:
        ser.rtscts = False
        ser.write_timeout = saved_wt
        try:
            ser.reset_output_buffer()
        except Exception:  # noqa: BLE001
            pass
        _reset_and_wait(tg, ice, record)
