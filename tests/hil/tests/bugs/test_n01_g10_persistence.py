"""N01: back-to-back idle G10 must not lose the next line to deferred EEPROM writes.

canonical_machine.c:445 runs persistence after G10 replies; xio.c:174 reverses
its serial-busy predicate. NIST RS274NGC v3 3.5.5 and TinyG wiki Coordinate-Systems
specify G10 coordinate updates and persistence after machining ends.
Use send_line deliberately: send_gcode adds a 50 ms pause which hides the defect.
"""
import pytest

pytestmark=[pytest.mark.persists,pytest.mark.dialect("d449"),
            pytest.mark.bug("N01",on=[])]

@pytest.mark.parametrize("rxm", [0,1])
def test_idle_g10_needs_only_its_response(tg, keep, rxm):
    tokens=[f"g{c}{a}" for c in range(54,60) for a in "xyz"]
    keep("rxm", *tokens)
    tg.set("rxm",rxm,allow="exercise both G10 receive paths")
    tg.send_gcode("G21 G90 G54",check=True)
    tg.wait_idle()
    try:
        # All coordinates change on each pass; do not let an unchanged EEPROM value
        # accidentally turn the persistence operation into a no-op.
        for repeat in range(4):
            for p in range(1,7):
                value=p+.125+repeat
                line=f"G10 L2 P{p} X{value} Y{-value} Z{value+.25}"
                tg.guard.check_gcode(line,allow="set coordinate offsets without motion")
                reply=tg.send_line(line,timeout=5)
                assert reply.status==0, (line,reply.raw)
        for p in range(1,7):
            value=p+3.125
            assert tg.get(f"g{53+p}x")==pytest.approx(value)
            assert tg.get(f"g{53+p}y")==pytest.approx(-value)
            assert tg.get(f"g{53+p}z")==pytest.approx(value+.25)
    finally:
        if tg.stat()==tg.dialect.stat("ALARM"):
            tg.clear_alarm()
