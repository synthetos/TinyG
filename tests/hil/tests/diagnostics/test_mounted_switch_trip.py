"""Diagnostic: what the firmware does when a mounted software switch trips mid-move."""
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.ice]


@pytest.mark.profile("switches-no")
def test_mounted_switch_trip_trace(tg, switches, ice, record):
    record("switch_config", {t: tg.get(t) for t in ("st", "xsn", "xsx")})
    switches.mount("x", "max", at=0.5)
    t0 = time.monotonic()
    tg.send_gcode("G91 G1 X1 F120", allow="1 mm X move across a software switch")
    time.sleep(2.5)
    switches.unmount_all()
    record("follower_error", repr(switches.error))
    record("trips", [(round(t - t0, 3), a, e, c, p) for t, a, e, c, p in switches.trips])
    record("samples", [(round(t - t0, 3), round(p, 4)) for t, _, p in switches.samples[::4]])
    record("serial_after_move", [(round(e.t - t0, 3) if hasattr(e, "t") else None, e.raw)
                                 for e in tg.drain(quiet=0.3)])
    record("hook", {f: ice.peek(f"test_hook.{f}") for f in ("magic", "pending", "axis", "bit", "action", "applied")})
    record("pc", [ice.where() for _ in range(8)])
    record("port_a", {r: ice.read(ice.board.port_reg("A", r), 1)[0] for r in ("OUT", "IN")})
    try:
        record("stat", tg.stat())
    except Exception as e:
        record("stat", repr(e))
