"""E164: in character mode the footer's byte count misses bytes that get no reply of their own, so a
host counting bytes (Grbl-style streaming, rxm=0 ex=0) drifts permanently.

controller.c _dispatch_command/_dispatch_control store readline()'s size for the line just read in
cs.linelen, which json_print_response() reports as f[2] and clears. The second byte of a CRLF ending
comes back as an empty line of size 1, and blank or whitespace-only lines are never answered, so
their bytes are never reported: one byte lost per CRLF line, every byte of a blank line lost. A
counting host then believes the ring is fuller than it is and eventually stops sending for good.

Each case is followed by an ordinary line, so bytes owed are reported by then: the replies' byte
counts must add up to exactly what was sent, with one reply per non-blank line.
"""
import time

import pytest

from tinyg_hil.protocol import Footer

pytestmark = [pytest.mark.dialect("d449"), pytest.mark.persists]

NEXT = b"G4 P0.01\n"
CASES = {
    "crlf": (b"G4 P0.01\r\n", 1),
    "blank-lf": (b"\n", 0),
    "blank-crlf": (b"\r\n", 0),
    "spaces-only": (b"   \n", 0),
    "tab-only": (b"\t\n", 0),
    "trailing-spaces-crlf": (b"G4 P0.01   \r\n", 1),
    "json-crlf": (b'{"fv":null}\r\n', 1),
    "double-lf": (b"G4 P0.01\n\n", 1),
    "file-of-crlf-lines": (b"G4 P0.01\r\n" * 20, 20),
}


@pytest.mark.bug("E164", on=["d449"])
@pytest.mark.parametrize("case", list(CASES))
def test_byte_counts_add_up(tg, keep, record, case):
    data, replies = CASES[case]
    keep("rxm", "ex")
    for token, value in (("rxm", 0), ("ex", 0)):
        if tg.get(token) != value:
            tg.set(token, value, allow="E164: character counting, no motion")
    tr = tg.transport
    tg.drain(quiet=0.1)
    mark = len(tr.history)
    tr.write(data + NEXT)
    end = time.monotonic() + 5
    while time.monotonic() < end and sum(1 for e in tr.history[mark:] if e.kind == "response") < replies + 1:
        time.sleep(0.05)
    tg.drain(quiet=0.3)
    footers = [Footer.from_event(e) for e in tr.history[mark:] if e.kind == "response"]
    counts = [f.count for f in footers if f]
    record("count", {"sent": len(data) + len(NEXT), "counts": counts, "statuses": [f.status for f in footers if f]})
    assert len(footers) == replies + 1, f"{len(footers)} replies for {replies + 1} lines"
    assert sum(counts) == len(data) + len(NEXT), f"reported {sum(counts)} of {len(data) + len(NEXT)} bytes: {counts}"
