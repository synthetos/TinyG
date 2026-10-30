"""Timing thresholds are timing-pending until external capture.

E002: M400 (wait for motion to finish) is missing on edge and wrong on master.

Edge: (uint8_t)400 == 144 falls to STAT_MCODE_COMMAND_UNSUPPORTED (gcode_parser.c:384/:400) and
_sync_to_planner (controller.c:434-440) has no drain state. Master answers M400 at once in JSON
mode, and in text mode releases it when the dwell's planner buffer is freed at dwell start
(planner.c:254-258), so `G4 P2` + `M400` returns immediately on both.
"""
import time

import pytest



def _text_mode(tg, line: str, timeout: float = 2.0):
    """Send one line in text mode; return (lines up to the prompt, prompt arrival time)."""
    tg.transport.write(line.encode() + b"\n")
    lines = []
    while True:
        ev = tg.wait_for(lambda e: True, timeout)
        lines.append(ev.raw)
        if ev.kind == "prompt":
            return lines, ev.t


@pytest.mark.timing
@pytest.mark.bug("E002", on=["d440"])
def test_m400_json_waits_for_dwell(tg):
    tg.send_gcode("G4 P2", check=True)
    t0 = time.monotonic()
    tg.transport.write(b"M400\n")
    time.sleep(0.2)
    tg.transport.write(b'{"sr":null}\n')     # control lines must still be answered during the wait
    responses = []
    try:
        while len(responses) < 2:
            ev = tg.wait_for(lambda e: e.kind == "response", 5.0)
            responses.append((ev.t - t0, ev))
    finally:
        tg.wait_idle()
    (t_first, first), (t_m400, m400) = responses
    assert "sr" in first.data.get("r", {}) and t_first < 1.0, first.raw
    assert m400.data["f"][1] == 0, m400.raw
    assert t_m400 >= 1.9


@pytest.mark.timing
@pytest.mark.bug("E002", on=["d440"])
def test_m400_text_mode_waits_for_dwell(tg):
    _text_mode(tg, "$fb")       # any $ line switches the board to text mode
    try:
        _text_mode(tg, "G4 P2")
        t0 = time.monotonic()
        lines, t_prompt = _text_mode(tg, "M400", timeout=5.0)
    finally:
        tg.get("fb")            # back to JSON mode
        tg.wait_idle()
    assert "ok>" in lines[-1], lines
    assert t_prompt - t0 >= 1.9


def test_m400_idle_acknowledges(tg):
    """Functional coverage, with no timing threshold or motion stimulus."""
    tg.wait_idle()
    response = tg.send_gcode("M400")
    assert response.status == 0, response.raw
