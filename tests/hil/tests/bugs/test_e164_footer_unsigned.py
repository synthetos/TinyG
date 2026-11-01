"""E164 follow-up: the character-mode footer prints its byte count with %d, so above 32767 bytes it goes
negative on the AVR (16-bit int).

json_parser.c json_print_response() formats cs.linelen (uint16_t) with "%d". Since E164 the count
includes bytes of lines that get no reply, so a long run of blank lines followed by one command can pass
32767. 33000 blank-line bytes go in with RTS/CTS flow control (so none are lost), then one ordinary
line: its footer must report all 33009 bytes.
"""
import time

import pytest

from tinyg_hil.protocol import Footer

pytestmark = [pytest.mark.dialect("d449"), pytest.mark.persists]

BLANK = 33000
LINE = b"G4 P0.01\n"


@pytest.mark.bug("E164", on=["d449"])
def test_footer_count_above_32767(tg, keep, record):
    keep("rxm", "ex")
    for token, value in (("rxm", 0), ("ex", 2)):
        if tg.get(token) != value:
            tg.set(token, value, allow="E164 follow-up: RTS/CTS so a long blank run is not lost")
    tr = tg.transport
    tr.ser.rtscts = True
    saved_timeout = tr.ser.write_timeout
    tr.ser.write_timeout = None
    try:
        tg.drain(quiet=0.1)
        mark = len(tr.history)
        tr.write(b"\n" * BLANK)
        tr.write(LINE)
        end = time.monotonic() + 15
        while time.monotonic() < end and not any(e.kind == "response" for e in tr.history[mark:]):
            time.sleep(0.05)
        tg.drain(quiet=0.3)
    finally:
        tr.ser.rtscts = False
        tr.ser.write_timeout = saved_timeout
    footers = [Footer.from_event(e) for e in tr.history[mark:] if e.kind == "response"]
    alarms = [e.raw for e in tr.history[mark:] if e.kind == "exception"]
    record("footer", {"counts": [f.count for f in footers if f], "alarms": alarms[:2]})
    assert not alarms, alarms[:2]
    assert len(footers) == 1, f"{len(footers)} replies"
    assert footers[0].count == BLANK + len(LINE), f"footer count {footers[0].count}"
