"""E167: a text-mode (MSG ...) is lost or repeated, depending on what the previous line left behind.

In text mode Gcode goes to gc_gcode_parser() without a fresh NV list (controller.c _dispatch_kernel),
cm_message() appends the message to the first free body slot, and text_response() prints a message
only from NV_BODY+1 (text_parser.c text_response). So the message prints only when the previous
command left exactly one object in the list ($fb does, which is why test_e058 passes). After a status
report or $sr it is appended further down and never printed (the W2 watch item); after another
message the old one still sits at NV_BODY+1 and is printed again instead of the new one. A blank
line (or the second byte of a CRLF ending) answers with text_response() too, repeating the message.

Wiki: Gcode-Language-Support (MSG comments are displayed to the operator).
"""
import time

import pytest

pytestmark = [pytest.mark.dialect("d449")]


def _command(tg, line):
    tg.transport.write(line.encode() + b"\n")
    deadline = time.monotonic() + 3
    lines = []
    while True:
        remaining = deadline - time.monotonic()
        assert remaining > 0, lines
        event = tg.wait_for(lambda e: True, remaining)
        lines.append(event.raw)
        if event.kind == "prompt":
            return "\n".join(lines)


@pytest.mark.bug("E167", on=[])
def test_message_after_status_report_text(tg):
    try:
        _command(tg, "?")
        assert "after-status" in _command(tg, "G21 (MSG after-status)")
        _command(tg, "$sr")
        assert "after-sr" in _command(tg, "G21 (MSG after-sr)")
    finally:
        tg.get("fb")


@pytest.mark.bug("E167", on=[])
def test_each_message_once_text(tg):
    try:
        _command(tg, "$fb")
        assert "first-msg" in _command(tg, "G21 (MSG first-msg)")
        second = _command(tg, "G21 (MSG second-msg)")
        assert "second-msg" in second and "first-msg" not in second, second
        plain = _command(tg, "G21")
        assert "msg" not in plain, plain
    finally:
        tg.get("fb")


@pytest.mark.bug("E167", on=[])
def test_message_not_repeated_by_blank_line_or_crlf_text(tg, keep):
    """Character mode: a blank line is answered with a prompt (line mode drops it without a reply)."""
    keep("rxm")
    tg.set("rxm", 0)
    try:
        _command(tg, "$fb")
        assert "blank-msg" in _command(tg, "G21 (MSG blank-msg)")
        assert "blank-msg" not in _command(tg, "")
        _command(tg, "$fb")
        tg.transport.write(b"G21 (MSG crlf-msg)\r\n")
        deadline, prompts = time.monotonic() + 1.0, []
        while time.monotonic() < deadline:
            event = tg.transport.next_event(deadline - time.monotonic())
            if event is not None and event.kind == "prompt":
                prompts.append(event.raw)
        assert sum("crlf-msg" in p for p in prompts) == 1, prompts
    finally:
        tg.get("fb")
