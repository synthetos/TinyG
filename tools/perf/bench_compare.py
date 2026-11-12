"""Paired bench evidence: never substitute host planned times or missing motors."""
from collections import defaultdict
import math
import random
import re
import json
from pathlib import Path
from decimal import Decimal, ROUND_HALF_UP
import statistics


LEGACY_PRECISION=json.loads((Path(__file__).resolve().parents[2]/'tests/hil/tinyg_hil/data/perf-449-print-precision.json').read_text())

def interval(values):
    """Deterministic paired bootstrap of median ratios, 95% percentile interval."""
    rng=random.Random(45004);n=len(values)
    draws=sorted(statistics.median(rng.choices(values,k=n)) for _ in range(5000))
    return [draws[124],draws[4874]]


def positive(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value) and value>0


def compare(records,expected=None,rt_cost=False):
    groups=defaultdict(list);expected=expected or {}
    for r in records:
        if r.get('kind')!='bench-perf':continue
        if r.get('perf_probes')=='on':continue
        if rt_cost and r.get('tag')!='B':continue
        if not rt_cost and r.get('rt')!='off':continue
        groups[(r['fixture'],r.get('bench'),r.get('source'))].append(r)
    output=[]
    signatures=defaultdict(set)
    for records_in_group in groups.values():
        for r in records_in_group:
            f=r.get('flash') or {}
            signatures[r.get('tag')].add((r.get('expected_fb'),f.get('hex_sha256'),f.get('commit')))

    for key,records in sorted(groups.items()):
        row=dict(fixture=key[0],bench=key[1],source=key[2],expect=expected.get(key[0],'same'),
                 result='PASS',findings=[],pairs=0,ratios={},steps=[],pulses=[],not_restorable=[],reduced_precision=[],rx_credits=[],explanations=[],attribution='unattributed')
        slots=defaultdict(dict);bad=row['findings'];failed=False
        if key[0].startswith('20_'):
            row['explanations'].append('449.01 can report stored END (stat 4) during motion/hold; A uses post-! zero velocity plus 0.5 s settle, B uses HOLD stat. N99 046666db fixes the stale-state reporting path; this does not waive failed captures or timing differences.')
        for tag in {r.get('tag') for r in records}:
            if len(signatures[tag])!=1:bad.append(f'tag {tag}: mixed firmware images/expectations across records')
        for r in records:
            serial=r.get('serial') or {}
            minimum=serial.get('rx_credit_min')
            row['rx_credits'].append(dict(slot=r.get('slot'),tag=r.get('tag'),initial=serial.get('rx_credit_initial'),minimum=minimum,last=serial.get('rx_credit_last')))
            if serial.get('probe_mode','250ms')!='off':
                if minimum is None:bad.append(f'{r.get("slot")}/{r.get("tag")}: missing receive-credit minimum')
                elif minimum<=2 or (isinstance(serial.get('rx_credit_initial'),int) and isinstance(serial.get('rx_credit_last'),int) and serial['rx_credit_last']<serial['rx_credit_initial']):bad.append(f'{r.get("slot")}/{r.get("tag")}: receive credit minimum {minimum}, final {serial.get('rx_credit_last')} (possible shrinking window); N55 priority probe credit loss on A is an expected difference, not a waiver')
            label=r.get('rt') if rt_cost else r.get('tag');slot=str(r.get('slot'))
            if label in slots[slot]:bad.append(f'duplicate {slot}/{label}')
            slots[slot][label]=r
        labels=('off','on') if rt_cost else ('A','B')
        ratios=defaultdict(list)
        for slot,pair in sorted(slots.items()):
            if set(pair)!=set(labels):bad.append(f'unpaired slot {slot}');continue
            a,b=(pair[k] for k in labels)
            if not a.get('valid') or not b.get('valid'):
                bad.append(f'slot {slot}: invalid capture: A={a.get("error")} B={b.get("error")}');continue
            if a.get('sha256')!=b.get('sha256') or a.get('identity',{}).get('id')!=b.get('identity',{}).get('id'):
                bad.append(f'slot {slot}: file or board identity differs');continue
            provenance_ok=True
            for r in (a,b):
                f=r.get('flash') or {}; sha=f.get('hex_sha256','')
                expected_fb=449.01 if r.get('tag')=='A' else r.get('expected_fb')
                actual=r.get('identity',{}).get('fb')
                if (not positive(expected_fb) or not positive(actual) or abs(actual-expected_fb)>.0005 or
                    not f.get('known') or not re.fullmatch('[0-9a-f]{64}',str(sha)) or
                    (r.get('expected_hex') and r['expected_hex']!=sha)):
                    bad.append(f'slot {slot}/{r.get("tag")}: firmware identity or flash provenance mismatch/unknown')
                    provenance_ok=False
            if not provenance_ok:continue
            same_image=a['flash']['hex_sha256']==b['flash']['hex_sha256']
            if same_image != rt_cost:
                bad.append(f'slot {slot}: A/B require distinct images; rt off/on requires the same image');continue
            fixed_a=a.get('fixed_snapshot') or {};fixed_b=b.get('fixed_snapshot') or {}
            if (not re.fullmatch('[0-9a-f]{64}',str(fixed_a.get('sha256',''))) or
                fixed_a.get('sha256')!=fixed_b.get('sha256') or
                not a.get('fixed_verified',{}).get('ok') or not b.get('fixed_verified',{}).get('ok')):
                bad.append(f'slot {slot}: fixed snapshot differs or readback was not verified');continue
            av,bv=a['settings']['values'],b['settings']['values']
            ignored={k for k in av.keys() | bv.keys() if re.fullmatch(r'ud[a-d][0-3]',k)}
            row['not_restorable']=sorted(set(row['not_restorable']) | ignored)
            av={k:v for k,v in av.items() if k not in ignored}
            bv={k:v for k,v in bv.items() if k not in ignored}
            # Only known B-only controls may be absent on A. Missing common
            # tokens are not evidence of identical settings.
            extra_b=bv.keys()-av.keys()
            allowed=lambda k: k.startswith('rt') or k in ('spph','spde','coph')
            missing=av.keys()-bv.keys()
            common=av.keys() & bv.keys()
            differences=[k for k in common if av[k]!=bv[k]]
            if not rt_cost and a['identity']['fb']==LEGACY_PRECISION['fb']:
                for check in a['fixed_verified'].get('reduced_precision',[]):
                    k=check.get('token');entry=LEGACY_PRECISION['tokens'].get(k)
                    if (k in differences and entry and check.get('precision')==entry['precision'] and
                        check.get('actual')==av[k] and check.get('expected')==bv[k] and
                        check.get('printed_expected')==av[k] and float(Decimal(str(bv[k])).quantize(Decimal(10)**-entry['precision'],rounding=ROUND_HALF_UP))==av[k]):
                        differences.remove(k)
                        row['reduced_precision']=sorted(set(row['reduced_precision'])|{k})
            if missing or any(not allowed(k) for k in extra_b) or not common or differences:
                bad.append(f'slot {slot}: settings differ/missing: '+str(sorted(set(differences)|missing|{k for k in extra_b if not allowed(k)})));continue
            if a['settings'].get('sr')!=b['settings'].get('sr'):
                bad.append(f'slot {slot}: status-report fields differ');continue
            am,bm=a['metrics']['motors'],b['metrics']['motors']
            if set(am)!=set(bm) or set(am)!= {'M1','M2','M3','M4'} or a.get('missing_motors') or b.get('missing_motors'):
                bad.append(f'slot {slot}: incomplete four-motor coverage');continue
            for m in am:
                for field in ('total','net'):
                    delta=bm[m][field]-am[m][field]
                    row['steps'].append(dict(slot=slot,motor=m,field=field,a=am[m][field],b=bm[m][field],delta=delta))
                    if field=='net' and abs(delta)>1:bad.append(f'{slot}/{m} {field} differs by {delta} steps');failed=True
                la,lb=am[m].get('reversal_legs'),bm[m].get('reversal_legs')
                row['steps'].append(dict(slot=slot,motor=m,field='reversal_legs',a=la,b=lb,delta=lb-la if isinstance(la,int) and isinstance(lb,int) else None))
                if not isinstance(la,int) or not isinstance(lb,int):bad.append(f'{slot}/{m}: reversal-leg evidence missing')
                elif la!=lb:bad.append(f'{slot}/{m}: reversal legs differ {la} vs {lb}');failed=True
                elif am[m]['total']!=bm[m]['total'] and abs(am[m]['net']-bm[m]['net'])<=1:
                    note='Total pulses differ with equal reversal legs and net within 1: incoming fractional-step phase can add one pulse per leg; see bench3-attribution/report.txt. Totals retained.'
                    if note not in row['explanations']:row['explanations'].append(note)
                if am[m]['total'] or bm[m]['total']:
                    pa,pb=am[m].get('min_pulse_s'),bm[m].get('min_pulse_s')
                    ra,rb=a.get('sample_rate_hz'),b.get('sample_rate_hz')
                    period=max(1/ra,1/rb) if positive(ra) and positive(rb) else None
                    row['pulses'].append(dict(slot=slot,motor=m,a=pa,b=pb,rate_a_hz=ra,rate_b_hz=rb,shorter_threshold_s=period,
                        max_rate_a=am[m].get('max_rate_hz'),max_rate_b=bm[m].get('max_rate_hz')))
                    if not positive(pa) or not positive(pb):bad.append(f'{slot}/{m}: pulse missing')
                    elif period is None:bad.append(f'{slot}/{m}: sample rate missing')
                    elif pa-pb>=period or math.isclose(pa-pb,period,rel_tol=1e-12,abs_tol=0):
                        bad.append(f'{slot}/{m}: pulse shorter by at least one sample period ({period}s)');failed=True
            metrics=['motion_s']
            if any(r['serial'].get('probe_mode')!='off' for r in (a,b)):
                bad.append(f'slot {slot}: A/B timing requires probes off on both builds (N55)');continue
            if a.get('parse_bound') or b.get('parse_bound'):
                if not (a.get('parse_bound') and b.get('parse_bound')):
                    bad.append(f'slot {slot}: parse-bound classification differs');continue
                metrics.append('lines_s')
            for metric in metrics:
                parent='metrics' if metric=='motion_s' else 'serial'
                x,y=a[parent].get(metric),b[parent].get(metric)
                if not positive(x) or not positive(y):bad.append(f'{slot}: missing/invalid {metric}');continue
                ratios[metric].append(y/x)
            row['pairs']+=1
        for metric,values in ratios.items():
            med=statistics.median(values);ci=interval(values)
            row['ratios'][metric]=dict(median=med,ci95=ci,n=len(values))
            limit=1.02 if rt_cost and metric=='motion_s' else 1.01 if metric=='motion_s' else .98 if metric=='lines_s' else 1.10
            violation=med<limit if metric=='lines_s' else med>limit
            uncertain=ci[0]<limit if metric=='lines_s' else ci[1]>limit
            if violation:bad.append(f'{metric} exceeds threshold {limit}');failed=True
            elif uncertain:bad.append(f'{metric} 95% interval crosses threshold {limit}')
        if row['pairs']<5:bad.append('need at least 5 paired slots for the 95% review')
        if not row['ratios']:bad.append('no comparable measurements')
        if bad:row['result']='FAIL' if failed else 'FINDING'
        elif row['ratios']['motion_s']['median']<.99:row['result']='FASTER-REVIEW'
        row['speed_review']='FASTER-REVIEW' if row['ratios'].get('motion_s',{}).get('median',1)<.99 else None
        if row['speed_review']:
            prefix=key[0].split('_')[0]
            row['attribution']='E180: preserved entry speed across freed planner buffers' if prefix=='05' else 'E089 a44844a5: startup lookahead before the first move; exact percentage also depends on stream timing and pulse phase' if prefix in ('14','16','18','19') else 'unattributed'
        output.append(row)
    return output


def round_trips(records):
    """Absolute B observations; no paired speed claim or A RTT baseline."""
    output=[]
    for r in records:
        if r.get('kind')!='bench-perf' or r.get('tag')!='B' or r.get('perf_probes')!='on':continue
        serial=r.get('serial') or {};bad=[]
        flash=r.get('flash') or {};actual=r.get('identity',{}).get('fb');expected=r.get('expected_fb')
        if not r.get('valid') or serial.get('completion')!='idle' or serial.get('errors'):
            bad.append('invalid or incomplete collection')
        if (not positive(actual) or not positive(expected) or abs(actual-expected)>.0005 or
            not flash.get('known') or not re.fullmatch('[0-9a-f]{64}',str(flash.get('hex_sha256',''))) or
            (r.get('expected_hex') and r['expected_hex']!=flash.get('hex_sha256'))):
            bad.append('firmware identity/provenance mismatch')
        if not r.get('fixed_verified',{}).get('ok') or not re.fullmatch('[0-9a-f]{64}',str(r.get('fixed_snapshot',{}).get('sha256',''))):
            bad.append('fixed settings not verified')
        if serial.get('probe_mode')!='250ms':bad.append('requires 250 ms probes')
        for k in ('stat_median_s','stat_p95_s'):
            if not positive(serial.get(k)):bad.append('missing '+k)
        initial,minimum,last=(serial.get('rx_credit_'+k) for k in ('initial','min','last'))
        if not all(isinstance(v,int) for v in (initial,minimum,last)):
            bad.append('missing receive-credit evidence')
        elif minimum<=2 or last<initial:bad.append('receive-credit pressure or shrinking window')
        output.append(dict(fixture=r['fixture'],slot=r.get('slot'),bench=r.get('bench'),rt=r.get('rt'),
            baseline='N55: no A streaming round-trip baseline; B absolute measurements only',
            stat_median_s=serial.get('stat_median_s'),stat_p95_s=serial.get('stat_p95_s'),
            rx_credit_initial=initial,rx_credit_min=minimum,rx_credit_last=last,
            result='FINDING' if bad else 'MEASURED',findings=bad))
    return output


def capture_evidence(record, bench_config=None):
    """Enrich old MSO records in memory; never change capture files or source records.

    The legacy map is supplied from bench configuration and accepted only when
    reconstructed total AND signed net match every recorded motor exactly.
    """
    if not record.get('valid') or record.get('source')!='mso':return record
    motors=record.get('metrics',{}).get('motors',{})
    if record.get('sample_rate_hz') and all('reversal_legs' in m for m in motors.values()):return record
    path=Path(record.get('capture_file',''))
    if not path.is_file():return record
    raw=json.loads(path.read_text())
    mapping=record.get('capture_channel_map')
    origin='record'
    if not mapping:
        config=bench_config or Path.home()/'.config/tinyg-hil/benches.json'
        if not Path(config).is_file():return record
        entries=json.loads(Path(config).read_text())
        candidates=[v for v in entries.values() if isinstance(v,dict) and v.get('name')==record.get('bench')]
        if len(candidates)!=1:return record
        mapping=candidates[0].get('mso',{}).get('digital');origin=str(config)
    if not mapping:return record
    names={name:key for key,name in mapping.items()};derived={}
    try:
        for motor,expected in motors.items():
            sk,dk=names[motor+' step'],names[motor+' dir']
            direction=raw['initial'][dk];j=0;last=None;legs=total=net=0
            dirs=raw['transitions'][dk]
            plus=1 if record['settings']['values'][motor[1:]+'po'] else 0
            for t,value in raw['transitions'][sk]:
                if value!=1:continue
                while j<len(dirs) and dirs[j][0]<=t:direction=dirs[j][1];j+=1
                if last is None or last!=direction:legs+=1
                last=direction;total+=1;net+=1 if direction==plus else -1
            if total!=expected['total'] or net!=expected['net']:return record
            derived[motor]=legs
    except KeyError:return record
    record['sample_rate_hz']=raw.get('rate')
    for motor,legs in derived.items():motors[motor]['reversal_legs']=legs
    record['derived_capture_evidence']=dict(file=str(path),channel_map_source=origin,verified='all motor total/net counts match')
    return record
