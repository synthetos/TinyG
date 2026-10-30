"""E140: writing se00-se29 stores the low 16 bits of the float as a cfgArray row with no bounds check;
a row past the table restarts the firmware on every status report and leaves the planner dead.

config_app.c:591-620 `_fp ... set_u16`; config.c:342 `*((uint16_t *)...) = (uint16_t)nv->value_int`
over value_flt's bits, so 524319.0f (0x490001F0) stores row 496 of a 407-row table. Stored indexes
are not validated on load (report.c:224-225) or when the status report is populated.
"""
import pytest

from tinyg_hil.protocol import Footer

# 524319.0f = 0x490001F0: stock 449.01 stores row 496, past the end of cfgArray
OUT_OF_RANGE = 524319


def _restarted(tg, events) -> bool:
    return any(e.kind == "banner" or "Loading configs" in e.raw for e in events)


def _watch_for_restart(tg, tries: int = 4) -> bool:
    """Send GETs (each can trigger a status report) and report whether a restart banner appeared."""
    for _ in range(tries):
        tg.transport.write(b'{"stat":null}\n')
        if _restarted(tg, tg.drain(quiet=0.8, limit=3.0)):
            return True
    return False


def _recover(tg, keys):
    """Set sr by name as the first command after a restart, then ^X to revive the planner."""
    line = ("{sr:{" + ",".join(f"{k}:t" for k in keys) + "}}\n").encode()
    for _ in range(4):
        tg.drain(quiet=0.3)
        tg.transport.write(line)
        events = tg.drain(quiet=0.8, limit=3.0)
        if any(e.kind == "response" for e in events) and not _restarted(tg, events):
            break
    tg.reset()


@pytest.mark.dialect("d449")
@pytest.mark.persists
@pytest.mark.resets_board
@pytest.mark.bug("E140", on=["d449"])
def test_out_of_range_se_index_rejected(tg):
    keys = list(tg.get("sr") or {})
    assert keys, "need the current status-report list to recover"
    restarted, status = True, None
    try:
        tg.drain(quiet=0.05)
        tg.transport.write(b'{"se00":%d}\n' % OUT_OF_RANGE)
        events = tg.drain(quiet=0.8, limit=3.0)
        status = next((Footer.from_event(e).status for e in events if e.kind == "response"), None)
        restarted = _restarted(tg, events) or _watch_for_restart(tg)
    finally:
        if restarted or status == 0:
            _recover(tg, keys)
    assert list(tg.get("sr") or {}) == keys
    assert not restarted
    assert status not in (None, 0)
