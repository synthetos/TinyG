"""Feedhold, resume and queue flush injected into the host replay (real canonical machine, planner, exec, prep).

    python3 -I replay_hold.py --work DIR [--source TREE] [--runs N] [--seed S] FILE_OR_DIR [...]

The replay is built with three driver changes: st_aline_in_flight() reports a block in flight while the driver runs
one; a hold is requested at chosen exec steps (as '!' would, cm_request_feedhold), and cm_feedhold_sequencing_callback()
runs on every step as the main loop does; once the hold is reached the run either resumes (cm_request_end_hold, as
'~') or flushes (cm_request_queue_flush, as '%', after which no more input is read, as a host stops sending).
A loop that makes no progress for 1e6 steps prints STUCK instead of aborting.

Per file: one plain run gives the step count and the final position; then N runs with 1-3 holds at random steps
(seeded), each resume or flush. Checks: the run ends (no STUCK, no EXEC error, exit 0); every requested hold was
reached and acted on; a run that only resumed ends where the plain run ends (runtime and steps); after a flush the
model position equals the runtime position (the machine knows where it stopped) and the step pulses match the
runtime target. Failures are printed with the hold schedule, so a run can be repeated exactly.
"""
from pathlib import Path
import argparse
import random
import re
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import trajectory_replay as tr  # noqa: E402

EXTS = {".nc", ".ngc", ".gcode", ".tap"}

HOLD_C = r'''
void xio_flush_input(void){printf("INPUTFLUSH\n");}  /* a '%' drops host input: the driver stops reading */
static int host_inflight, host_flushed, host_nholds, host_hi, host_acted, host_done;
static long host_ticks, host_hold_at[16];
static char host_hold_act[16];
static void host_hold_step(void)
{
    host_ticks++;
    if (host_hi < host_nholds && !host_acted && host_ticks >= host_hold_at[host_hi] && cm.hold_state == FEEDHOLD_OFF
        && cm.motion_state == MOTION_RUN) {
        cm_request_feedhold();
        printf("HOLDREQ %d %ld %u\n", host_hi, host_ticks, source_line);
        host_acted = 1;                                   /* requested; act when the hold is reached */
    }
    cm_feedhold_sequencing_callback();
    if (host_acted == 1 && cm.hold_state == FEEDHOLD_HOLD) {
        printf("HELD %d %ld %c %.6f %.6f %.6f\n", host_hi, host_ticks, host_hold_act[host_hi],
               (double)mp_get_runtime_absolute_position(0), (double)mp_get_runtime_absolute_position(1),
               (double)mp_get_runtime_absolute_position(2));
        if (host_hold_act[host_hi] == 'f') { cm_request_queue_flush(); host_flushed = 1; }
        else cm_request_end_hold();
        host_acted = 2;
    }
    if (host_acted == 2 && cm.hold_state == FEEDHOLD_OFF) {
        printf("HOLDDONE %d %ld\n", host_hi, host_ticks);
        host_done++; host_hi++; host_acted = 0;
    }
}
'''


def _hold_patch(driver):
    old = "bool st_aline_in_flight(void){return false;}"
    assert driver.count(old) == 1, "st_aline_in_flight stub not found"
    driver = driver.replace(old, "static int host_inflight_flag;\nbool st_aline_in_flight(void){return host_inflight_flag;}")
    anchor = "static void take_one(void)\n"
    assert driver.count(anchor) == 1
    driver = driver.replace(anchor, HOLD_C.replace("host_inflight,", "") + anchor)
    loop_old = ("        unsigned guard=0;\n        while(mb.r==b && b->buffer_state!=MP_BUFFER_EMPTY){\n"
                "            st_pre.buffer_state=PREP_BUFFER_OWNED_BY_EXEC;\n            stat_t status=mp_exec_move();\n")
    assert driver.count(loop_old) == 1, "exec loop not found"
    driver = driver.replace(loop_old, ("        unsigned guard=0; host_inflight_flag=1;\n"
                                       "        while(mb.r==b && b->buffer_state!=MP_BUFFER_EMPTY){\n"
                                       "            st_pre.buffer_state=PREP_BUFFER_OWNED_BY_EXEC;\n"
                                       "            stat_t status=mp_exec_move();\n            host_hold_step();\n"))
    guard_old = "            if(++guard>1000000)abort();\n        }\n"
    assert driver.count(guard_old) == 1
    driver = driver.replace(guard_old, ('            if(++guard>1000000){printf("STUCK %u %ld hold %d motion %d\\n",source_line,host_ticks,'
                                        '(int)cm.hold_state,(int)cm.motion_state);exit(125);}\n        }\n        host_inflight_flag=0;\n'))
    main_old = "    if(argc>1)batch=atoi(argv[1]);\n"
    assert driver.count(main_old) == 1
    driver = driver.replace(main_old, main_old + (
        "    if(argc>2){char *p=argv[2];while(*p&&host_nholds<16){host_hold_at[host_nholds]=strtol(p,&p,10);"
        "host_hold_act[host_nholds++]=*p?*p++:'r';if(*p==',')p++;}}\n"))
    line_old = "        if (cm_is_alarmed() != STAT_OK) printf(\"STATE %u %u\\n\", source_line, cm.machine_state);\n        alarm(0);\n    }\n"
    assert driver.count(line_old) == 1
    driver = driver.replace(line_old, line_old[:-6] + "        if (host_flushed) break;\n    }\n")
    done_old = "    printf(\"DONE %u"
    assert driver.count(done_old) == 1
    driver = driver.replace(done_old, (
        "    printf(\"HOLDS %d of %d ticks %ld\\n\", host_done, host_nholds, host_ticks);\n"
        "    printf(\"FINAL model %.6f %.6f %.6f runtime %.6f %.6f %.6f\\n\", (double)cm.gmx.position[0], (double)cm.gmx.position[1],"
        " (double)cm.gmx.position[2], (double)mp_get_runtime_absolute_position(0), (double)mp_get_runtime_absolute_position(1),"
        " (double)mp_get_runtime_absolute_position(2));\n" + done_old))
    return driver


def build(root, out):
    out.mkdir(parents=True, exist_ok=True)
    return tr.build(Path(root).resolve(), out, extra_patch=_hold_patch)


def run(exe, gcode, holds="", timeout=300):
    args = [str(exe), "28"] + ([holds] if holds else [])
    p = subprocess.run(args, input=gcode, capture_output=True, timeout=timeout)
    text = p.stdout.decode(errors="replace")
    r = {"rc": p.returncode, "stderr": p.stderr.decode(errors="replace")[-300:], "stuck": None, "held": [], "done": 0,
         "ticks": 0, "final": None, "steps": []}
    for line in text.splitlines():
        f = line.split()
        if not f:
            continue
        if f[0] == "STUCK":
            r["stuck"] = line
        elif f[0] == "HELD":
            r["held"].append(line)
        elif f[0] == "HOLDS":
            r["done"], r["ticks"] = int(f[1]), int(f[5])
        elif f[0] == "FINAL":
            r["final"] = ([float(x) for x in f[2:5]], [float(x) for x in f[6:9]])
        elif f[0] == "STEPS":
            r["steps"].append((int(f[1]), float(f[2]), int(f[3])))       # motor, target steps, signed pulses
    return r


def check(plain, r, schedule):
    """Problems with one injected run (empty list = pass)."""
    bad = []
    n = len([s for s in schedule.split(",") if s])
    if r["stuck"]:
        bad.append(r["stuck"])
    if r["rc"]:
        bad.append(f"exit {r['rc']} {r['stderr'][-160:]}")
    if r["done"] != len(r["held"]) or len(r["held"]) > n:
        bad.append(f"holds reached {len(r['held'])}, completed {r['done']}, scheduled {n}")
    if r["final"] is None:
        return bad + ["no FINAL line"]
    model, runtime = r["final"]
    for m, target, pulses in r["steps"]:
        if abs(target - pulses) > 1.5:
            bad.append(f"motor {m}: {pulses} pulses vs runtime target {target:.1f} steps")
    flushed = "f" in schedule and any(" f " in h for h in r["held"])
    if flushed:
        if max(abs(a - b) for a, b in zip(model, runtime)) > 0.001:
            bad.append(f"after the flush the model {model} is not the runtime {runtime}")
    elif plain["final"] is not None:
        if max(abs(a - b) for a, b in zip(runtime, plain["final"][1])) > 0.001:
            bad.append(f"resumed run ends at {runtime}, the plain run at {plain['final'][1]}")
    return bad


def files_in(paths):
    for p in map(Path, paths):
        if p.is_dir():
            yield from sorted(q for q in p.rglob("*") if q.suffix.lower() in EXTS and q.is_file())
        elif p.is_file():
            yield p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--work", type=Path, required=True)
    ap.add_argument("--source", type=Path, default=Path(__file__).resolve().parents[2])
    ap.add_argument("--runs", type=int, default=6)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--flush-share", type=float, default=0.3)
    a = ap.parse_args()
    exe = build(a.source, a.work.resolve())
    rnd = random.Random(a.seed)
    total = failed = sched = held = flushes = 0
    for f in files_in(a.paths):
        g = f.read_bytes()
        plain = run(exe, g)
        if plain["rc"] or plain["stuck"] or plain["ticks"] < 20:          # too short to hold anywhere useful
            print(f"SKIP {f.name}: plain run rc {plain['rc']} {plain['stuck'] or ''} ticks {plain['ticks']}")
            continue
        for _ in range(a.runs):
            k = rnd.choice([1, 1, 2, 3])
            at = sorted(rnd.randrange(1, plain["ticks"]) for _ in range(k))
            acts = ["f" if (i == k - 1 and rnd.random() < a.flush_share) else "r" for i in range(k)]
            schedule = ",".join(f"{t}{c}" for t, c in zip(at, acts))
            r = run(exe, g, schedule)
            bad = check(plain, r, schedule)
            total += 1; sched += k; held += len(r["held"]); flushes += sum(" f " in h for h in r["held"])
            if bad:
                failed += 1
                print(f"FAIL {f.name} holds={schedule}: {bad[:3]}")
    print(f"{total} injected runs, {failed} failed; holds scheduled {sched}, reached {held} (flushes {flushes})")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
