"""Desk-only timing of the real held-block planner on a full 32-block queue.

Run: python3 tests/host/measure_hold_replan.py
Reports native host time, NOT AVR time or an interrupt-latency bound. Physical
execution is stubbed by the planner-startup fixture; planner/zoid math is real.
"""
import json
import platform
from pathlib import Path
import subprocess
import tempfile
from source_extract import function

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / 'firmware/tinyg'
FIXTURE = Path(__file__).parent / 'planner_startup'


def main():
    prefix = (FIXTURE / 'check.c').read_text().split('int main(void) {', 1)[0]
    dispatch = function((FW / 'plan_exec.c').read_text(), r'(?m)^stat_t mp_exec_move\([^;{}\n]*\)\s*')
    code = '#include <time.h>\n' + prefix + r'''
static double stamp(void) {
    struct timespec t; assert(clock_gettime(CLOCK_MONOTONIC, &t)==0);
    return t.tv_sec + t.tv_nsec * 1e-9;
}
int main(void) {
    const float lengths[] = {0.1f, 1.0f, 10.0f};
    for(unsigned workload=0; workload<3; workload++) {
        for(unsigned sample=0; sample<1000; sample++) {
            setup();
            for(unsigned i=0; i<PLANNER_BUFFER_POOL_SIZE; i++) move((i+1)*lengths[workload]);
            assert(mb.buffers_available==0);
            cm.motion_state=MOTION_HOLD; cm.hold_state=FEEDHOLD_HOLD;
            mpBuf_t *bf=mp_get_run_buffer();
            bf->length *= 0.5f;  // held part way through the first move
            bf->entry_vmax=0; bf->block_state=BLOCK_INITIALIZING;
            assert(mp_get_last_buffer()==bf->pv);  // every queued block is in the planner walk
            const double begin=stamp();
            for(unsigned repeat=0; repeat<100; repeat++) mp_replan_held_block();
            const double elapsed=(stamp()-begin)/100;
            assert((SREG&128) && cm.hold_state==FEEDHOLD_HOLD && mb.buffers_available==0);
            assert(bf->entry_velocity==0);
            printf("%.1f %.9f\n", lengths[workload], elapsed);
        }
    }
    return 0;
}
''' + dispatch
    with tempfile.TemporaryDirectory(prefix='hold-replan-') as directory:
        src=Path(directory)/'bench.c'; exe=Path(directory)/'bench'
        src.write_text(code)
        command=['cc','-std=gnu99','-O2','-funsigned-char','-D_POSIX_C_SOURCE=200809L',
                 '-include','tgmath.h','-I'+str(FIXTURE),'-I'+str(FW),str(src),
                 *[str(FW/f) for f in ('planner.c','plan_line.c','plan_zoid.c','util.c')],
                 '-lm','-o',str(exe)]
        subprocess.run(command,check=True)
        output=subprocess.check_output([str(exe)],text=True)
    results={}
    for line in output.splitlines():
        length, duration=line.split()
        results.setdefault(length,[]).append(float(duration)*1e6)
    summary={}
    for length, values in results.items():
        values.sort()
        summary[length]={'samples':len(values),'mean_us':sum(values)/len(values),
                         'median_batch_mean_us':values[len(values)//2],'p95_batch_mean_us':values[int(len(values)*.95)],
                         'max_batch_mean_us':max(values)}
    print(json.dumps({'method':'native -O2, 100 replans per monotonic-clock sample; 1000 samples per length',
                      'platform':platform.platform(),'machine':platform.machine(),
                      'queue_blocks':32,'feed_mm_min':300,'length_mm':summary,
                      'limitation':'Host estimate only; no AVR cycle or interrupt blackout bound.'},indent=2))


if __name__=='__main__':
    main()
