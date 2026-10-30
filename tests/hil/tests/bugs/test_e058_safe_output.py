"""E058: literal percent text must never be used as a printf format string.

Serial coverage requires the E021 input-framing fix; without it the RX ISR strips
percent bytes before the reporting code sees them. Host tests isolate the writers.
"""
import time

import pytest

MESSAGE = "50% %s %n %%"


@pytest.mark.bug("E058", on=["d440"])
def test_percent_format_sequences_json(tg):
    response = tg.send_gcode(f"G21 (MSG {MESSAGE})", check=True)
    assert response.body.get("msg", "").strip() == MESSAGE


@pytest.mark.bug("E058", on=["d440"])
def test_percent_format_sequences_text(tg):
    def command(line):
        tg.transport.write(line.encode() + b"\n")
        deadline = time.monotonic() + 3
        lines = []
        while True:
            remaining = deadline - time.monotonic()
            assert remaining > 0, lines
            event = tg.wait_for(lambda e: True, remaining)
            lines.append(event.raw)
            if event.kind == "prompt":
                return lines
    command("$fb")
    try:
        lines = command(f"G21 (MSG {MESSAGE})")
        assert MESSAGE in "\n".join(lines), lines
    finally:
        tg.get("fb")
