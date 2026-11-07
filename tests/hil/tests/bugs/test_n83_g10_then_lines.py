"""N83 (#298): lines sent right behind a G10 at rest must all arrive, and the offset must still be saved.

cm_set_coord_offsets() wrote EEPROM at rest, before the reply (N01). Each write leaves only high-level interrupts on
and serial RX is medium level, so bytes arriving meanwhile are lost: status 13 and ALARM. A G-code file streamed ahead
of the replies does this with its first G10 (seen: a Haas file on bench 3, 2 of 2 streams). The gap between lines
matters: 0 is one burst, 2 ms and more leave the RX ring empty when the G10 runs while the next line is on its way.
TinyG wiki Coordinate-Systems: G10 offsets are saved to EEPROM once the machining cycle is done.

The follow-up lines are M5, so nothing moves and nothing queues. test_g10_saved_after_the_lines resets the board:
the reset is the observation, as in test_n68.
"""
import time

import pytest

pytestmark = [pytest.mark.persists, pytest.mark.resets_board, pytest.mark.dialect("d449"), pytest.mark.timeout(120),
              pytest.mark.bug("N83", on=["d449"])]

WHY = "N83 offset at rest, no motion"
FOLLOW = [f"N{n} M5" for n in range(30, 110, 10)]


def _burst(tg, z, gap):
    """G10 then FOLLOW with `gap` seconds between writes; returns the exception reports and how many replies came."""
    line = f"G10 L2 P1 Z{z:.3f}"
    tg.guard.check_gcode(line, allow=WHY)
    mark = len(tg.transport.history)
    if gap:
        for ln in [line] + FOLLOW:
            tg.transport.write((ln + "\n").encode())
            time.sleep(gap)
    else:
        tg.transport.write((line + "\n" + "".join(ln + "\n" for ln in FOLLOW)).encode())
    time.sleep(1.0)
    tg.drain()
    h = tg.transport.history[mark:]
    return [e.raw for e in h if e.kind == "exception"], sum(1 for e in h if e.kind == "response")


@pytest.mark.parametrize("gap_ms", [0, 2, 5, 10, 20], ids=lambda g: f"gap-{g}ms")
def test_lines_behind_a_g10_all_arrive(tg, keep, record, gap_ms):
    keep("g54z")
    tg.send_gcode("G21 G90", check=True)
    z = tg.get("g54z")
    exc, replies = _burst(tg, z - 1.25, gap_ms / 1000.0)
    stat = tg.stat()
    record("n83", {"gap_ms": gap_ms, "stat": stat, "exceptions": exc[:3], "replies": replies})
    print("N83", gap_ms, stat, replies, exc[:1])
    if stat == tg.dialect.stat("ALARM"):
        tg.clear_alarm()
    assert not exc and stat != tg.dialect.stat("ALARM"), f"input lost at {gap_ms} ms: {exc[:2]}"
    assert replies == 1 + len(FOLLOW), f"{replies} replies to {1 + len(FOLLOW)} lines"


def test_g10_saved_after_the_lines(tg, keep, record):
    keep("g54z")
    tg.send_gcode("G21 G90", check=True)
    z = tg.get("g54z")
    want = z - 2.5
    exc, _ = _burst(tg, want, 0)
    if tg.stat() == tg.dialect.stat("ALARM"):
        tg.clear_alarm()
    tg.reset()
    saved = tg.get("g54z")
    record("n83_saved", {"want": want, "saved": saved, "exceptions": exc[:2]})
    assert not exc and saved == pytest.approx(want), f"saved {saved}, want {want}"
