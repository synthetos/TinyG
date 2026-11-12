"""Compare host planned durations, not board performance or UART throughput.

python3 tools/perf/compare.py A.json B.json [--expected fixture-expectations.json]
Only identical files/settings are paired. Refusals, missing records and mismatches
are findings, never zero-duration wins. Expected notes explain differences but do
not waive errors or slowdowns without review.
"""
import argparse
import json
import math
from pathlib import Path
import sys

# Isolated Python omits the script directory; resolve only our own sibling modules.
sys.path.insert(0, str(Path(__file__).resolve().parent))

EXPECTED = {
    'E029': 'G18 direction corrected',
    'N87': 'short final G1 reaches its target',
    'N80/N105/N78': '449 error-20 cases: no valid A timing',
    'N104': 'missing arc offsets: alarm on A, refusal on B',
    'E060': 'excluded from A/B; spindle pause measured separately',
}


def compare(a, b, expected=None):
    expected = expected or {}
    if a['profile'] != b['profile']:
        raise ValueError('machine profiles differ')
    def index(data):
        rows = {}
        for row in data['files']:
            if row['name'] in rows:
                raise ValueError('duplicate fixture: ' + row['name'])
            rows[row['name']] = row
        return rows
    aa, bb = index(a), index(b)
    if not aa or not bb:
        raise ValueError('empty replay')
    rows = []
    for name in sorted(aa.keys() | bb.keys()):
        x, y = aa.get(name), bb.get(name)
        row = dict(name=name, a=None, b=None, delta_pct=None,
                   expect=expected.get(name, 'same'), result='FINDING')
        if x is None or y is None:
            row['detail'] = 'missing A or B record'
        elif x['sha256'] != y['sha256']:
            row['detail'] = 'fixture contents differ'
        elif 'error' in x or 'error' in y:
            row['detail'] = 'A: ' + x.get('error', 'ok') + '; B: ' + y.get('error', 'ok')
        else:
            ta, tb = x['host']['duration_s'], y['host']['duration_s']
            if not all(isinstance(t, (float, int)) and math.isfinite(t) and t > 0 for t in (ta, tb)):
                row['detail'] = 'invalid or zero planned duration'
            else:
                row.update(a=ta, b=tb, delta_pct=100*(tb/ta-1),
                           result=('SLOWER' if tb > ta*1.01 else
                                   'FASTER-REVIEW' if tb < ta*.99 else 'OK'), detail='')
        rows.append(row)
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('a', type=Path, nargs='?'); ap.add_argument('b', type=Path, nargs='?')
    ap.add_argument('--bench', type=Path, nargs='+', help='bench record files or run directories')
    ap.add_argument('--rt-cost', action='store_true', help='pair B rt off/on instead of A/B')
    ap.add_argument('--rtt-only', action='store_true', help='absolute B-only streaming round trips, no A baseline (N55)')
    ap.add_argument('--expected', type=Path)
    args = ap.parse_args()
    notes = json.loads(args.expected.read_text()) if args.expected else {}
    if args.bench:
        from bench_compare import compare as bench_compare, capture_evidence
        records=[]
        try:
            for path in args.bench:
                paths=path.rglob('perf-*.json') if path.is_dir() else [path]
                for p in paths:
                    data=json.loads(p.read_text())
                    if data.get('kind')=='bench-perf': records.append(capture_evidence(data))
            if args.rtt_only:
                if args.rt_cost:ap.error('--rtt-only cannot pair --rt-cost')
                from bench_compare import round_trips
                rows=round_trips(records)
                print('B ABSOLUTE STREAMING RTT: N55 prevents an A baseline; no paired RTT claim')
                for row in rows:print(json.dumps(row))
                if not rows:print('FINDING: no B probe records')
                return int(not rows or any(r['result']!='MEASURED' for r in rows))
            rows=bench_compare(records,notes,args.rt_cost)
        except (ValueError, KeyError, TypeError) as e:
            ap.exit(1, 'Invalid bench record: '+str(e)+'\n')
        print('BENCH PAIRED RATIOS: B/A' if not args.rt_cost else 'BENCH PAIRED RATIOS: B rt on/off')
        print('N55: no A streaming RTT baseline. Use --rtt-only for B absolute results.')
        print('fixture | pairs | motion ratio [95%] | lines/s ratio | stat median/p95 ratios | result | expect')
        for r in rows:
            v=r['ratios']; motion=v.get('motion_s',{})
            def ratio(k): return f"{v[k]['median']:.4f}" if k in v else 'n/a'
            print(f"{r['fixture']} | {r['pairs']} | {ratio('motion_s')} {motion.get('ci95',[])} | "
                  f"{ratio('lines_s')} | {ratio('stat_median_s')}/{ratio('stat_p95_s')} | {r['result']} | {r['expect']}")
            print('  max absolute net step difference:', max((abs(x['delta']) for x in r['steps'] if x['field']=='net'),default=None))
            print('  step totals, nets and reversal legs:',json.dumps(r['steps']))
            if r['speed_review']:print('  motion FASTER-REVIEW; attribution: '+r['attribution'])
            print('  pulse widths and maximum rates by motor:', json.dumps(r['pulses']))
            if r['reduced_precision']: print('  A readback at reduced precision: '+', '.join(r['reduced_precision']))
            if r['not_restorable']: print('  not-restorable user words (excluded): '+', '.join(r['not_restorable']))
            for credit in r['rx_credits']: print('  receive credits: '+str(credit))
            for explanation in r['explanations']: print('  '+explanation)
            for finding in r['findings']: print('  '+finding)
        if not rows: print('FINDING: no paired bench records')
        return int(not rows or any(r['result']!='PASS' for r in rows))
    if args.a is None or args.b is None:
        ap.error('provide A.json B.json or --bench records/directories')
    a, b = (json.loads(p.read_text()) for p in (args.a, args.b))
    try:
        rows = compare(a, b, notes)
    except (ValueError, KeyError) as e:
        ap.exit(1, str(e)+'\n')
    print(f"HOST PLANNED TIME ONLY: A={a['firmware']['commit']} B={b['firmware']['commit']}")
    print('fixture                              A seconds  B seconds   B/A change  result  expect')
    for r in rows:
        times = f"{r['a']:10.3f} {r['b']:10.3f} {r['delta_pct']:+10.2f}%" if r['a'] is not None else '       n/a        n/a         n/a'
        print(f"{r['name']:36} {times} {r['result']:8} {r['expect']}")
        if r['detail']: print('  ' + r['detail'][:600])
    print('Expected-difference review list (not blanket waivers):')
    for bug, note in EXPECTED.items(): print(f'  {bug}: {note}')
    print('Input gaps, operator holds, spindle delays, parse rate and stat latency need the bench.')
    return int(any(r['result'] != 'OK' for r in rows))


if __name__ == '__main__':
    raise SystemExit(main())
