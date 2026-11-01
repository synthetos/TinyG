"""E163: XON/XOFF leaves too little headroom for a USB-serial host, so a host that honours XOFF still
overruns the RX ring.

xio_usart.h XOFF_RX_HI_WATER_MARK is 80% of the 250-byte ring: the board sends XOFF with 50 bytes left.
A host only sees the XOFF after the USB-serial adapter forwards it (the FTDI latency timer is 16 ms,
up to ~180 bytes at 115200), and whatever it wrote meanwhile still arrives. Streaming files with
XON/XOFF handled by the application (as terminal programs do) overran the ring on bench 2 (E162 then
alarmed with status 13). Flow control the host honours must not lose input.

No motion: dwells fill the planner so the controller reads slowly, then dwell lines are sent one at a
time in char mode with XON/XOFF on, the host pausing whenever the last flow byte from the board was
XOFF.

XON/XOFF over USB serial stays best-effort: the adapter can hold the XOFF for its 16 ms latency timer,
longer than the 175 bytes of headroom take at 115200, and a 250-byte ring leaves no more room. So a
clean stream is required in one of ATTEMPTS tries (the old 80% mark lost 140 of 150 lines every time;
the 30% mark loses input in about 1 in 9 tries). An attempt may fail only by the lost-input alarm
(status 13, E162); a short or failed attempt without it, or any other exception, fails the test. Hosts should use line mode or RTS/CTS.
"""
import time

import pytest

from tinyg_hil.protocol import Footer

pytestmark = [pytest.mark.dialect("d449"), pytest.mark.persists]

XOFF, XON = 0x13, 0x11
FILL = 40                                    # paced dwells: more than the 32 planner buffers
LINES = 150
LINE = "G4 P0.001 (xon headroom check)"      # 30 characters: a typical CAM line length
ATTEMPTS = 3


def paused(tr):
    return bool(tr.flow) and tr.flow[-1][1] == XOFF


def _attempt(tg):
    """Fill the planner, stream LINES honouring XOFF, and report what came back."""
    tr = tg.transport
    for _ in range(FILL):
        tg.send_gcode("G4 P0.1", check=True, timeout=5.0)
    tg.drain(quiet=0.05)
    mark, flow_mark = len(tr.history), len(tr.flow)
    deadline = time.monotonic() + 60
    for _ in range(LINES):
        while paused(tr):                                   # the application honours XOFF
            assert time.monotonic() < deadline, "no XON from the board"
            time.sleep(0.001)
        tr.write((LINE + "\n").encode())
    end = time.monotonic() + 30
    while time.monotonic() < end:
        got = sum(1 for e in tr.history[mark:] if e.kind == "response")
        if got >= LINES or any(e.kind == "exception" for e in tr.history[mark:]):
            break
        time.sleep(0.2)
    tg.drain(quiet=1.0, limit=10.0)
    events = tr.history[mark:]
    footers = [Footer.from_event(e) for e in events if e.kind == "response"]
    alarms = [e.data["er"] for e in events if e.kind == "exception" and isinstance(e.data.get("er"), dict)]
    bad = [f.status for f in footers if f and f.status != 0]
    xoffs = sum(1 for _, b in tr.flow[flow_mark:] if b == XOFF)
    return {"answered": len(footers), "bad_statuses": sorted(set(bad)), "bad": len(bad),
            "alarms": alarms[:3], "xoff_sent": xoffs,
            "clean": not alarms and not bad and len(footers) == LINES}


@pytest.mark.bug("E163", on=[])
def test_host_honouring_xoff_loses_nothing(tg, keep, record):
    keep("rxm", "ex")
    for token, value in (("rxm", 0), ("ex", 1)):
        if tg.get(token) != value:
            tg.set(token, value, allow="E163: XON/XOFF flow control, no motion")
    attempts = []
    try:
        for _ in range(ATTEMPTS):
            result = _attempt(tg)
            attempts.append(result)
            assert result["xoff_sent"], "the board never sent XOFF: the check did not exercise flow control"
            if result["clean"]:
                break
            lost = [a for a in result["alarms"] if a.get("st") == 13]
            other = [a for a in result["alarms"] if a.get("st") != 13]
            assert lost and not other, f"an attempt failed other than by a lost-input alarm (status 13): {result}"
            tg.drain(quiet=0.3)
            if tg.stat() == tg.dialect.stat("ALARM"):
                tg.clear_alarm()
            tg.wait_idle(timeout=20)
    finally:
        record("xoff_headroom", {"sent": LINES, "attempts": attempts})
        tg.drain(quiet=0.3)
        if tg.stat() == tg.dialect.stat("ALARM"):
            tg.clear_alarm()
        tg.wait_idle(timeout=20)
    assert attempts[-1]["clean"], f"no clean stream in {ATTEMPTS} attempts while honouring XOFF: {attempts}"
