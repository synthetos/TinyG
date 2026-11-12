"""Segment trajectory replay: the real planner, exec and segment prep (step_replay), printing every prepared segment.

Each prepared segment prints "SEG <dt_min> <steps m0> <steps m1> <steps m2> <steps m3>" (dt in minutes, steps as
the prep received them, before DDA correction). From these, per-motor velocity, acceleration and jerk can be
estimated across segments and checked against the profile, and step totals against the runtime targets.
This is the base of the live feed override battery: the same checks must hold with override changes injected.

Profile (from step_replay): X on motors 1 and 2, Z on motor 3, A on motor 4; 1280 steps/mm; XYZ feed and
velocity 1200 mm/min; axis jerk 500 (x 1e6 mm/min^3).
"""
from pathlib import Path
import math
import subprocess

import step_replay

STEPS_PER_MM = 1280.0
FEED_MAX = 1200.0                 # mm/min, XYZ
JERK_MAX = 500e6                  # mm/min^3


def build(root, out, extra_patch=None):
    exe = step_replay.build(root, out)
    engine = (out / "step_engine.c").read_text()
    marker = "stat_t st_prep_line(float t[],float e[],float dt){"
    assert engine.count(marker) == 1, "st_prep_line wrapper not found"
    engine = engine.replace(marker, marker + 'printf("SEG %.9g %.9g %.9g %.9g %.9g %.9g %.9g %lu %d\\n",(double)dt,(double)t[0],'
                                              '(double)t[1],(double)t[2],(double)t[3],(double)mr.segment_velocity,(double)mb.r->jerk,'
                                              '(unsigned long)mr.gm.linenum,(int)mr.section);')
    engine = engine.replace('(int)mr.section);', '(int)mr.section);printf("BLK %.9g %.9g %.9g %.9g %.9g %.9g %.9g %d %d %d %.9g %.9g %u\\n",(double)mb.r->entry_velocity,'
                            '(double)mb.r->cruise_velocity,(double)mb.r->exit_velocity,(double)mb.r->length,'
                            '(double)mb.r->exit_vmax,(double)mb.r->cruise_vmax,(double)mb.r->delta_vmax,(int)mb.r->gm.motion_mode,'
                            '(int)mb.r->nx->buffer_state,(int)mb.r->nx->block_type,(double)mb.r->nx->entry_vmax,'
                            '(double)mb.r->nx->braking_velocity,(unsigned)(mb.r->fl_tags|fl_profile_tags(mb.r->naiive_move_time,'
                            'mb.r->cruise_velocity,mb.r->cruise_vmax)));', 1)
    if not (Path(root) / 'firmware/tinyg/feedlimit.h').exists():
        engine = engine.replace('mb.r->fl_tags|fl_profile_tags(mb.r->naiive_move_time,mb.r->cruise_velocity,mb.r->cruise_vmax)', '0')
    (out / "step_engine.c").write_text(engine)
    # The planner holds a lone first move for PLANNER_STARTUP_MS waiting for lookahead; the host clock never
    # advances, so release it as the firmware's own callback would once the time has passed.
    driver = (out / "driver.c").read_text()
    old = "static void finish(void)\n{\n"
    assert driver.count(old) == 1, "finish() not found"
    if 'mp_startup_callback' in (Path(root) / 'firmware/tinyg/planner.h').read_text():
        driver = driver.replace(old, old + "    rtc.sys_ticks += 1024; mp_startup_callback();\n")
    # The arc generator queues a chord only with PLANNER_BUFFER_HEADROOM buffers free; drain to that, not just
    # to the replay's batch level, or a long arc waits forever.
    arc = "        while (arc.run_state != BLOCK_IDLE && cm_is_alarmed() == STAT_OK) {\n            room();\n"
    assert driver.count(arc) == 1, "arc loop not found"
    driver = driver.replace(arc, "        while (arc.run_state != BLOCK_IDLE && cm_is_alarmed() == STAT_OK) {\n"
                                 "            room(); while (mb.buffers_available < PLANNER_BUFFER_HEADROOM && mp_has_runnable_buffer()) drain_one();\n")
    # The driver frees dwell blocks unrun; report each one's time (microseconds) and source line for timing.
    # edge-next keeps the time in bf->dwell_seconds, older trees in bf->gm.move_time (seconds either way).
    other = "    } else {\n        if (mp_free_run_buffer()) cm_cycle_end();\n    }\n"
    assert driver.count(other) == 1, "take_one free branch not found"
    secs = "b->dwell_seconds" if "dwell_seconds" in (Path(root) / "firmware/tinyg/planner.h").read_text() else "b->gm.move_time"
    driver = driver.replace(other, "    } else {\n"
                            f'        if (b->block_type == BLOCK_TYPE_DWELL) printf("DWELL %.9g %u\\n",(double){secs}*1e6,'
                            "(unsigned)block_line[b - mb.bf]);\n"
                            "        if (mp_free_run_buffer()) cm_cycle_end();\n    }\n")
    if extra_patch:
        driver = extra_patch(driver)
    (out / "driver.c").write_text(driver)
    import json
    args = json.loads((out / "command.json").read_text())
    args[args.index("-lm"):args.index("-lm")] = [str(out / (n + ".c")) for n in ["plan_exec", "kinematics", "step_engine"]]
    args.insert(1, "-DF_CPU=32000000UL")
    p = subprocess.run(args, capture_output=True, text=True)
    (out / "trajectory_build.log").write_text(p.stdout + p.stderr)
    if p.returncode:
        raise RuntimeError(p.stderr[-4000:])
    return exe


def run(exe, gcode, batch=28, timeout=60):
    p = subprocess.run([str(exe), str(batch)], input=gcode.encode(), capture_output=True, timeout=timeout)
    if p.returncode:
        raise RuntimeError(p.stderr.decode()[-2000:])
    if 'EXCEPTION' in p.stderr.decode():
        raise RuntimeError(p.stderr.decode()[-2000:])
    segs, steps = [], []
    for line in p.stdout.decode().splitlines():
        f = line.split()
        if f and (f[0] in ('STATE', 'ARC_STATUS', 'EXEC', 'STUCK', 'BAD') or
                  (f[0] in ('LINE', 'JSON', 'TEXT') and int(f[2]) not in (0, 3))):
            raise RuntimeError('replay rejected program: ' + line)
        if f and f[0] == "SEG":
            segs.append((float(f[1]), [float(x) for x in f[2:6]], float(f[6]), float(f[7]), int(f[8]), int(f[9])))
        elif f and f[0] == "STEPS":
            steps.append(f)
    return segs, steps


def kinematics(segs, motor):
    """Per-segment average velocity (mm/min) of one motor and the acceleration/jerk estimates between segments."""
    v = [s[1][motor] / STEPS_PER_MM / s[0] for s in segs if s[0] > 0]
    dt = [s[0] for s in segs if s[0] > 0]
    a = [(v[i + 1] - v[i]) / ((dt[i] + dt[i + 1]) / 2) for i in range(len(v) - 1)]
    j = [(a[i + 1] - a[i]) / ((dt[i] + 2 * dt[i + 1] + dt[i + 2]) / 4) for i in range(len(a) - 1)]
    return v, a, j


def worst(segs):
    """The largest |velocity| and |jerk| ratio to the profile over XYZ motors (motor 2 is Z, motors 0/1 are X)."""
    vmax = jmax = 0.0
    for m in (0, 2):
        v, a, j = kinematics(segs, m)
        if v:
            vmax = max(vmax, max(abs(x) for x in v) / FEED_MAX)
        if j:
            jmax = max(jmax, max(abs(x) for x in j) / JERK_MAX)
    return vmax, jmax


def finite(segs):
    return all(math.isfinite(s[0]) and all(math.isfinite(x) for x in s[1]) for s in segs)


def path_jerk(segs):
    """Tangential jerk estimates from the exec's segment velocities, within runs of one line number: the largest
    ratio to that block's jerk, and the largest speed step between consecutive segments of different lines
    (a junction): (jerk_ratio, junction_step_mm_min)."""
    worst_j, worst_step = 0.0, 0.0
    for i in range(len(segs) - 2):
        (d0, _, v0, _, n0, _), (d1, _, v1, j1, n1, _), (d2, _, v2, _, n2, _) = segs[i], segs[i + 1], segs[i + 2]
        if n0 == n1 == n2 and j1 > 0 and min(d0, d1, d2) > 0:
            a01 = (v1 - v0) / ((d0 + d1) / 2)
            a12 = (v2 - v1) / ((d1 + d2) / 2)
            jerk = (a12 - a01) / ((d0 + 2 * d1 + d2) / 4)
            worst_j = max(worst_j, abs(jerk) / j1)
    for i in range(len(segs) - 1):
        if segs[i][4] != segs[i + 1][4]:
            worst_step = max(worst_step, abs(segs[i + 1][2] - segs[i][2]))
    return worst_j, worst_step
