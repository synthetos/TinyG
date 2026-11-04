"""Paired A/B analysis of perfbench captures: G-<A|B>-j<jerk>-<slot>-edges.json.

Per run: cutting time per section (split at the Z plunges and lifts), total cutting time, net steps per
motor over the whole capture (perfbench ends where it starts, so all must be 0), and measured step pulse
widths (high time per step channel). Pairs are adjacent slots (1,2), (3,4), ... in alternating order.
Gain = A time / B time - 1 on total cutting time; reported as the median and a 95% t interval of the mean.
"""
import json, math, os, re, statistics as st, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze import motor_steps, MOTORS

PREFIX = os.environ.get('PAIR_PREFIX', 'G')
T95 = {1: 12.71, 2: 4.30, 3: 3.18, 4: 2.78, 5: 2.57, 6: 2.45, 7: 2.36, 8: 2.31, 9: 2.26}


def run_metrics(path):
    d = json.load(open(path))
    steps = {ax: motor_steps(d, *MOTORS[ax]) for ax in MOTORS}
    z = steps['Z']; up = z[0][1]
    runs, cur = [], [z[0]]
    for a, b in zip(z, z[1:]):
        if b[0] - a[0] > 0.05:
            runs.append(cur); cur = [b]
        else:
            cur.append(b)
    runs.append(cur)
    plunges = [r[-1][0] for r in runs if r[0][1] != up]
    lifts = [r[0][0] for r in runs if r[0][1] == up]
    sections = []
    for p in plunges:
        nxt = [l for l in lifts if l > p]
        if nxt:
            sections.append(nxt[0] - p)
    net = {ax: sum(s for _, s in steps[ax]) for ax in steps}
    pulses = {}
    for ax, (stp, _) in MOTORS.items():
        tr = d['transitions'].get(stp, []); level = d['initial'][stp]; rise = None; w = []
        for t, v in tr:
            if v == 1 and level == 0:
                rise = t
            elif v == 0 and level == 1 and rise is not None:
                w.append((t - rise) * 1e6)
            level = v
        if w:
            pulses[ax] = (round(min(w), 2), round(st.median(w), 2))
    return {'sections': [round(s, 3) for s in sections], 'total': round(sum(sections), 3), 'net': net,
            'pulses': pulses}


def main(folder):
    files = sorted(f for f in os.listdir(folder) if re.match(PREFIX + r'-[AB]-j\d+-\d+-edges\.json$', f))
    runs = {}
    for f in files:
        fw, jerk, slot = re.match(PREFIX + r'-([AB])-j(\d+)-(\d+)-edges', f).groups()
        runs[(int(jerk), int(slot))] = (fw, run_metrics(os.path.join(folder, f)))
    for jerk in sorted({j for j, _ in runs}):
        print(f'== jerk {jerk}')
        slots = sorted(s for j, s in runs if j == jerk)
        for s in slots:
            fw, m = runs[(jerk, s)]
            print(f"  slot {s:2} {fw}  total {m['total']:7.3f}s  sections {m['sections']}  net {m['net']}  pulse min/med us {m['pulses']}")
        gains = []
        for a, b in zip(slots[0::2], slots[1::2]):
            x, y = runs[(jerk, a)], runs[(jerk, b)]
            ta = x[1]['total'] if x[0] == 'A' else y[1]['total']
            tb = y[1]['total'] if y[0] == 'B' else x[1]['total']
            gains.append(ta / tb - 1)
        if len(gains) >= 2:
            mean, sd = st.mean(gains), st.stdev(gains)
            half = T95.get(len(gains) - 1, 2.0) * sd / math.sqrt(len(gains))
            print(f"  paired gain (A/B - 1): median {st.median(gains) * 100:.2f}%  mean {mean * 100:.2f}%  "
                  f"95% CI [{(mean - half) * 100:.2f}%, {(mean + half) * 100:.2f}%]  n={len(gains)}  {['%.2f%%' % (g * 100) for g in gains]}")
        for sec in range(3):
            a = [m['sections'][sec] for (j, s), (fw, m) in runs.items() if j == jerk and fw == 'A' and len(m['sections']) > sec]
            b = [m['sections'][sec] for (j, s), (fw, m) in runs.items() if j == jerk and fw == 'B' and len(m['sections']) > sec]
            if a and b:
                print(f"  section {'ABC'[sec]}: A median {st.median(a):.3f}s  B median {st.median(b):.3f}s  ({(st.median(a) / st.median(b) - 1) * 100:+.2f}%)")


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__)))
