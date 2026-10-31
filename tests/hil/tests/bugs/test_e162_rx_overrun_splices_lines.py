"""E162: bytes tossed on an RX ring overrun splice lines together, and the result runs if it parses.

xio_usb.c USB_RX_ISR_vect drops an incoming byte when the 250-byte ring is full and says nothing.
The reader (_gets_helper, xio_usart.c) then joins the head of one line to the tail of a later one.
Streaming the dense goose program without flow control ran 'G1 X16.8717 Y119' (soft limit) and
drove Y 583 mm on bench 2. The fix records where input was lost and fails every line that lost
bytes, or follows lost bytes, with the existing overflow alarm (ALARM 13, 'Buffer full non-fatal',
controller.c _dispatch_command); such a line never runs.

No motion: dwells fill the planner so the controller stops reading lines, then a burst of dwell
lines with flow control off overruns the ring. Char mode answers each line with its length + 1 in
the footer (f[2]), so a spliced line that ran shows up as an OK response whose length no sent line has.
"""
import time

import pytest

from tinyg_hil.protocol import Footer

pytestmark = [pytest.mark.dialect("d449"), pytest.mark.persists]

FILL = 40                                    # paced dwells: more than the 32 planner buffers
BURST = 60                                   # dwell lines sent at once: ~600 bytes against a 250-byte ring


def burst_lines():
    """Dwells of 7 to 11 characters, so most splices change the length."""
    return [f"G4 P0.{'0' * (k % 5)}{k % 9 + 1}" for k in range(BURST)]


@pytest.mark.bug("E162", on=["d449"])
def test_overrun_lines_alarm_instead_of_running(tg, keep, record):
    keep("rxm", "ex")
    for token, value in (("rxm", 0), ("ex", 0)):             # char mode, no flow control: nothing paces the host
        if tg.get(token) != value:
            tg.set(token, value, allow="E162: flow control off for an overrun, no motion")
    lines = burst_lines()
    try:
        for _ in range(FILL):
            tg.send_gcode("G4 P0.1", check=True, timeout=5.0)  # answered once queued; the planner fills
        tg.drain(quiet=0.05)
        mark = len(tg.transport.history)
        tg.transport.write("".join(line + "\n" for line in lines).encode())
        time.sleep(FILL * 0.1 + 1.0)                          # every dwell has run
        tg.drain(quiet=1.0, limit=20.0)
        events = tg.transport.history[mark:]
        responses = [Footer.from_event(e) for e in events if e.kind == "response"]
        alarms = [e.data["er"] for e in events if e.kind == "exception" and isinstance(e.data.get("er"), dict)]
        lengths = {len(line) + 1 for line in lines}
        spliced_ran = [f.count for f in responses if f and f.status == 0 and f.count not in lengths]
        record("rx_overrun", {"sent": BURST, "responses": len(responses), "alarms": alarms[:5],
                              "spliced_ran": spliced_ran[:10],
                              "statuses": sorted({f.status for f in responses if f})})
        if len(responses) >= BURST:
            pytest.skip("the burst did not overrun the RX ring")
        assert not spliced_ran, f"spliced lines ran: footer lengths {spliced_ran[:10]}"
        assert any(a.get("st") == tg.dialect.status("BUFFER_FULL") for a in alarms), \
            f"lost input raised no ALARM 13: {alarms[:3]}"
    finally:
        tg.drain(quiet=0.3)
        if tg.stat() == tg.dialect.stat("ALARM"):
            tg.clear_alarm()
        tg.wait_idle(timeout=15)
