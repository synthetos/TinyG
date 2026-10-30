"""E001: M114 (get position) and M115 (firmware info), which OpenPnP relies on, are missing on edge.

The M-code switch (gcode_parser.c:384-400) ends at `case 51` and falls to
STAT_MCODE_COMMAND_UNSUPPORTED (132); canonical_machine.h:161 has no GET_POSITION/GET_FIRMWARE
next_action. Master 440.21 prints `X:.. Y:..` and `FIRMWARE_NAME:TinyG, ...` lines.
"""
import re

import pytest

AXES = "XYZABC"
M114_LINE = re.compile(r"^" + r"\s+".join(rf"{a}:(-?\d+\.\d{{4}})" for a in AXES) + r"\s*$")


def _m114_position(resp) -> dict:
    """Position from a JSON-mode M114 reply: r.posx.. (proposed fix) or a raw `X:..` line (master)."""
    if "posx" in resp.body:
        return {a: resp.body[f"pos{a.lower()}"] for a in AXES}
    for line in resp.text():
        m = M114_LINE.match(line)
        if m:
            return {a: float(v) for a, v in zip(AXES, m.groups())}
    raise AssertionError(f"no position in M114 reply: {resp.raw} / {resp.text()}")


def _text_mode(tg, line: str, timeout: float = 2.0) -> list[str]:
    """Send one line in text mode and return its output up to the prompt (board stays in text mode)."""
    tg.transport.write(line.encode() + b"\n")
    lines = []
    while True:
        ev = tg.wait_for(lambda e: True, timeout)
        lines.append(ev.raw)
        if ev.kind == "prompt":
            return lines


@pytest.mark.bug("E001", on=["d449"])
def test_m114_json(tg):
    tg.send_gcode("G92 X12.5 Y-3", check=True)
    try:
        resp = tg.send_gcode("M114")
        assert resp.status == 0
        pos = _m114_position(resp)
        assert pos["X"] == pytest.approx(12.5, abs=1e-3)
        assert pos["Y"] == pytest.approx(-3, abs=1e-3)
        assert pos["Z"] == pytest.approx(tg.get("posz"), abs=1e-3)
    finally:
        tg.send_gcode("G92.1", check=True)


@pytest.mark.bug("E001", on=["d449"])
def test_m114_text_mode(tg):
    _text_mode(tg, "$fb")       # any $ line switches the board to text mode
    try:
        lines = _text_mode(tg, "M114")
    finally:
        tg.get("fb")            # back to JSON mode
    assert "ok>" in lines[-1], lines
    assert any(M114_LINE.match(line) for line in lines), lines


@pytest.mark.bug("E001", on=["d449"])
def test_m115_json(tg):
    resp = tg.send_gcode("M115")
    assert resp.status == 0
    if "fb" in resp.body:
        assert resp.body["fb"] == pytest.approx(tg.identity["fb"])
    else:
        text = "\n".join(resp.text())
        assert "FIRMWARE_NAME:TinyG" in text
        assert f"FIRMWARE_BUILD:{tg.identity['fb']:.2f}" in text
