"""A requested feed factor must cover short lines and arcs as well as long lines."""
import time
import pytest
from tinyg_hil import sender

pytestmark = [pytest.mark.bug("N111", on=["d449"]), pytest.mark.dialect("d449"),
              pytest.mark.motion, pytest.mark.persists, pytest.mark.timeout(180)]

PATTERNS = {
    "long4": (["G1 Y20 F600"] * 4, "G0 Y-80"),
    "short200": (["G1 Y0.4 F600"] * 200, "G0 Y-80"),
    "arc": (["G17 G2 X0 Y20 I0 J10 F600"], "G0 Y-20"),
}

@pytest.mark.parametrize("pattern", PATTERNS)
def test_feed_factor_covers_job(tg, keep, record, pattern):
    keep("ej", "rxm", "ex", "qv", "sv", "si", "xvm", "xfr", "yvm", "yfr")
    for key,value in dict(ej=1,rxm=1,ex=1,qv=1,sv=1,si=100,
                          xvm=1200,xfr=1200,yvm=1200,yfr=1200).items():
        tg.set(key,value,allow="feed override job timing")
    tg.set("mfoe",1)
    tg.set("m48e",1)
    tg.send_gcode("G21 G91 G94 G64 M48",check=True,allow="relative feed test")
    lines,back=PATTERNS[pattern]
    for line in lines+[back]:
        tg.guard.check_gcode(line,allow="feed override job timing")
    times=[]
    try:
        for factor in (1,.5):
            tg.wait_idle(timeout=60)
            tg.set("mfo",factor)
            start=time.monotonic()
            run_lines = [line.replace("F600", "F300") for line in lines] if factor == 1 else lines
            result=sender.stream(tg,run_lines,"line",timeout=90)
            assert result.ok, result
            tg.wait_idle(timeout=90)
            times.append(time.monotonic()-start)
            tg.set("mfo",1)
            tg.send_gcode(back,check=True,allow="return to start")
            tg.wait_idle(timeout=60)
    finally:
        tg.set("mfo",1)
        tg.send_gcode("G90",check=True)
    ratio=times[1]/times[0]
    record("n111_job_time",dict(pattern=pattern,halved_feed_s=times[0],half_s=times[1],ratio=ratio))
    # Compare equivalent programmed speeds; host/status latency is looser than MSO.
    assert .9 <= ratio <= 1.1, (pattern,times,ratio)
