"""Bench (switchbox + MSO): what the spindle and coolant outputs do when the machine stops, on the real lines.

E019's tests prove the output registers through the ICE. This reads the lines themselves: the switchbox reads the
spindle-enable and coolant lines, the MSO measures the spindle PWM. For each stop the spindle runs (M3 S<mid>)
with coolant on (M8), the stop is triggered, and the lines are read again:

  alarm, job kill (^D), limit switch hit (a real edge on X min), reset (^X), shutdown, M2, M30:
      spindle enable and coolant go low, the PWM falls to the off phase ($p1pof) at the same frequency.
  feedhold: recorded only. Edge keeps the spindle running in a hold (spindle pause is E060, not merged).

NIST RS274NGC 3.6.1 (M2/M30 stop the spindle and coolant); TinyG wiki Alarm Processing (alarm, shutdown and
job kill stop spindle and coolant). Needs a bench-file entry with "switchbox" (port, inputs, outputs) and "mso".
"""
import time

import pytest

from tinyg_hil import benches, mso
from tinyg_hil.switchbox import SwitchBox

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.resets_board,
              pytest.mark.dialect("d449"), pytest.mark.timeout(900)]

OUTPUTS = "spindle and coolant outputs on the bring-up bench (no motors)"


def _connect(sbcfg):
    """The ESP32-S2 box often misses the first getInfo after the port was last closed (DTR drops on close): retry."""
    for attempt in range(3):
        try:
            return SwitchBox(sbcfg["port"], lines=sbcfg.get("lines", "asserted"), allow_reset=True,
                             purpose="outputs on stop", lock_wait=30).connect()
        except Exception:  # noqa: BLE001
            if attempt == 2:
                raise
            time.sleep(1.0)


def _lines(sb, outputs):
    """Output line levels. The ESP32-S2 box can miss a request or answer late; retry until a state-shaped reply."""
    for attempt in range(4):
        try:
            st = sb.state()
            if isinstance(st.get("channels"), list):
                lv = {c["name"]: c["level"] for c in st["channels"]}
                return {k: lv[v] for k, v in outputs.items()}
        except Exception:  # noqa: BLE001
            if attempt == 3:
                raise
        time.sleep(0.5)
    raise AssertionError("switchbox gave no state-shaped reply in four tries")


def _pwm(cfg, name):
    with mso.Capture(cfg, analog=[name], seconds=1.0, rate=1_000_000, pwm_window=0.2) as cap:
        pass
    rows = cap.pwm(name)[1:]                   # five 0.2 s windows; the first can straddle the capture start
    return {"duty": round(sum(r["duty"] for r in rows) / len(rows), 4), "freq_hz": rows[0]["freq_hz"]}


def _stop_alarm(tg, sb, inputs):
    tg.send_json({"alarm": None})


def _stop_kill(tg, sb, inputs):
    tg.send_raw(b"\x04")


def _stop_limit(tg, sb, inputs):
    for attempt in range(3):
        try:
            sb.pulse_channel(inputs["xmin"], ms=100)
            return
        except Exception:  # noqa: BLE001
            if attempt == 2:
                raise
            time.sleep(0.5)


def _stop_reset(tg, sb, inputs):
    tg.reset()


def _stop_shutdown(tg, sb, inputs):
    tg.send_json({"shutd": None})


def _stop_m2(tg, sb, inputs):
    tg.send_gcode("M2", allow=OUTPUTS)


def _stop_m30(tg, sb, inputs):
    tg.send_gcode("M30", allow=OUTPUTS)


def _stop_feedhold(tg, sb, inputs):
    line = "G1 X20 F100"                       # a 12 s move, so the hold lands mid-move
    tg.guard.check_gcode(line, allow=OUTPUTS)
    tg.send_line(line)
    time.sleep(1.0)
    tg.send_raw(b"!")


STOPS = [("alarm", _stop_alarm, True), ("job kill", _stop_kill, True), ("limit hit", _stop_limit, True),
         ("reset", _stop_reset, True), ("shutdown", _stop_shutdown, True), ("M2", _stop_m2, True),
         ("M30", _stop_m30, True), ("feedhold", _stop_feedhold, False)]


def _recover(tg):
    try:
        if tg.stat() == tg.dialect.stat("ALARM"):
            tg.clear_alarm()
            return
    except Exception:  # noqa: BLE001
        pass
    tg.reset()                                 # shutdown and a stuck hold need a reset


def test_outputs_on_stop(tg, keep, record):
    bench = benches.bench(tg.transport.port)
    sbcfg, msocfg = bench.get("switchbox") or {}, bench.get("mso") or {}
    if not sbcfg.get("outputs") or not msocfg.get("analog"):
        pytest.skip("needs a bench with switchbox outputs and an MSO PWM channel")
    pwm_name = msocfg["analog"]["0"] if "0" in msocfg["analog"] else next(iter(msocfg["analog"].values()))
    keep("xsn", "p1pof")
    tg.set("xsn", 3, allow="X min as homing and limit, for the limit-hit stop")
    tg.reset()
    phase_off = tg.get("p1pof")
    lo, hi = tg.get("p1csl"), tg.get("p1csh")
    speed = (lo + hi) / 2
    want_on = round(tg.get("p1cpl") + (tg.get("p1cph") - tg.get("p1cpl")) / 2, 4)
    sb = _connect(sbcfg)
    results, bad = [], []
    try:
        sb.release_all()
        for name, stop, must_stop in STOPS:
            _recover(tg)
            tg.send_gcode("G90 G21", allow=OUTPUTS)
            tg.send_gcode("G28.3 X0", allow="declare X for the feedhold move")
            tg.send_gcode(f"M3 S{speed:g}", check=True, allow=OUTPUTS)
            tg.send_gcode("M8", check=True, allow=OUTPUTS)
            time.sleep(0.6)
            before, pwm_before = _lines(sb, sbcfg["outputs"]), _pwm(msocfg, pwm_name)
            stop(tg, sb, sbcfg["inputs"])
            time.sleep(1.0)
            after, pwm_after = _lines(sb, sbcfg["outputs"]), _pwm(msocfg, pwm_name)
            try:
                stat = tg.stat()
            except Exception:  # noqa: BLE001
                stat = None
            r = {"stop": name, "before": before, "pwm_before": pwm_before, "after": after, "pwm_after": pwm_after,
                 "stat_after": stat, "expected_on_duty": want_on, "phase_off": phase_off}
            on_ok = before["spindle_enable"] == 1 and before["coolant"] == 1 and abs(pwm_before["duty"] - want_on) < 0.006
            off_ok = (after["spindle_enable"] == 0 and after["coolant"] == 0 and
                      abs(pwm_after["duty"] - phase_off) < 0.006)
            r["ok"] = on_ok and (off_ok if must_stop else True)
            results.append(r)
            if not r["ok"]:
                bad.append(r)
            if name == "feedhold":
                tg.send_raw(b"%")              # discard the held move
                time.sleep(0.5)
            tg.send_gcode("M5 M9", allow=OUTPUTS)
    finally:
        sb.release_all()
        sb.disconnect()
        _recover(tg)
        tg.send_gcode("M5 M9", allow=OUTPUTS)
    record("outputs_on_stop", {"speed": speed, "results": results})
    for r in results:
        print("STOP", r["stop"], "ok" if r["ok"] else "BAD", r["before"], r["pwm_before"], "->", r["after"],
              r["pwm_after"], "stat", r["stat_after"])
    assert not bad, f"outputs wrong around a stop: {[ (b['stop'], b['after'], b['pwm_after']) for b in bad]}"
