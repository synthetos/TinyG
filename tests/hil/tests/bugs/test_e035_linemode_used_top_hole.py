"""E035: in line mode, freeing a control line that is the newest RX buffer leaves a FREE hole at
used_top, so the next line is written over a queued line or hidden behind the hole (stall/reorder).

xio.c:618-624 (_free_processed_buffer) frees used_top but only moves used_top down when pv is
FREE, which it never is; xio.c:461-464 (_get_free_buffer) then places the next line at
used_base->bufp + size + 1, on top of the second queued line, and the scans at xio.c:560-561 and
575-576 stop at the hole. Trigger: a control line read while all 12 RX headers hold G-code that is
waiting for planner space. Line mode ($rxm=1) is the 449 default; 440 has no line mode.
"""
import time

import pytest

from tinyg_hil import NoResponse

pytestmark = pytest.mark.dialect("d449")


def _fill_planner(tg):
    """Queue a long dwell, then short dwells until the planner stops accepting lines."""
    tg.send_gcode("G4 P6", check=True)
    for _ in range(40):
        try:
            tg.send_gcode("G4 P0.05", timeout=0.4)
        except NoResponse:
            return
    pytest.fail("planner never reached its headroom limit")


@pytest.mark.resets_board
@pytest.mark.bug("E035", on=["d449"])
def test_control_line_while_rx_buffers_full(tg):
    try:
        _fill_planner(tg)
        tg.transport.write(b"G4 P0.05\n" * 14)      # fills the 12 RX headers; a few wait in the USART ring
        time.sleep(0.3)
        # Complete lines in the ring: each is read in the same call that allocates its buffer.
        tg.transport.write(b'N101 G4 P0.01\nN102 G4 P0.01\n{"sr":null}\nN103 G4 P0.01\n')
        ns, statuses, sr_seen, end = [], [], False, time.monotonic() + 30
        while 103 not in ns:
            ev = tg.transport.next_event(end - time.monotonic())
            if ev is None:
                break
            if ev.kind != "response":
                continue
            if "sr" in ev.body:
                sr_seen = True
            elif "n" in ev.body:
                ns.append(ev.body["n"])
                statuses.append(ev.data["f"][1])
        assert ns == [101, 102, 103], ns
        assert statuses == [0, 0, 0]
        assert sr_seen
    finally:
        tg.reset()
