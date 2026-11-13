"""Offline-testable bench metrics and single-reader line-mode collection."""
import hashlib
import json
import math
import time
from collections import deque
from pathlib import Path
from .sender import _footer, RESERVE


def options(env):
    tag, slot, rt = env.get('FIRMWARE_TAG'), env.get('SLOT'), env.get('RT', 'off')
    if tag not in ('A', 'B') or not slot or rt not in ('off', 'on'):
        raise ValueError('set FIRMWARE_TAG=A|B, SLOT=<paired slot>, RT=off|on')
    if tag == 'A' and rt != 'off':
        raise ValueError('449.01 has no rt: A requires RT=off')
    return tag, slot, rt


def probe_options(env):
    _,ms=probe_parameters(env)
    mode=env.get('PERF_PROBES','off')
    if mode not in ('off','on'):raise ValueError('PERF_PROBES must be off|on')
    if mode=='on' and env.get('FIRMWARE_TAG')!='B':
        raise ValueError('PERF_PROBES=on requires B: N55 prevents an A streaming RTT baseline')
    return ms/1000 if mode=='on' else None


def probe_parameters(env):
    kind=env.get('PERF_PROBE_KIND','stat')
    ms=float(env.get('PERF_PROBE_MS','250'))
    if kind not in ('stat','qr') or not math.isfinite(ms) or ms<=0:
        raise ValueError('PERF_PROBE_KIND must be stat|qr and PERF_PROBE_MS finite and positive')
    return kind,ms


def probe_suffix(env):
    interval=probe_options(env)
    kind,ms=probe_parameters(env)
    return f"{'off' if interval is None else 'on'}-{kind}-{ms:g}ms"


def capture_budget(source, requested, timeout):
    budget=min(requested,500) if source=='mso' else requested
    timeout=budget-2 if timeout is None else float(timeout)
    if not (math.isfinite(budget) and 2<timeout<budget):
        raise ValueError('capture must outlast stream; MSO at 2 MHz is limited to 500 s')
    return budget,timeout


def digest(lines):
    return hashlib.sha256(('\n'.join(lines)+'\n').encode()).hexdigest()


def percentile(values, q):
    if not values: return None
    v=sorted(values); n=(len(v)-1)*q; lo=int(n); hi=min(lo+1,len(v)-1)
    return v[lo]+(v[hi]-v[lo])*(n-lo)


def steps(edges, initial, plus):
    motors, first, last = {}, [], []
    for motor, level in plus.items():
        sk, dk = f'{motor} step', f'{motor} dir'
        if any(k not in edges or k not in initial for k in (sk,dk)):
            raise ValueError(f'missing capture channel: {motor}')
        se,de=edges[sk],edges[dk]
        legs=0;last_direction=None
        if initial[sk] != 0 or (se and se[-1][1] != 0):
            raise ValueError(f'{motor} capture cuts a pulse')
        rises,widths,net,j,direction,rise=[],[],0,0,initial[dk],None
        for t,v in se:
            if v == 1:
                while j<len(de) and de[j][0]<=t: direction=de[j][1];j+=1
                if last_direction is None or direction!=last_direction:legs+=1
                last_direction=direction
                rises.append(t);net+=1 if direction==level else -1;rise=t
            elif rise is not None: widths.append(t-rise);rise=None
        intervals=[b-a for a,b in zip(rises,rises[1:])]
        if rises: first.append(rises[0]);last.append(rises[-1])
        motors[motor]=dict(total=len(rises),net=net,reversal_legs=legs,min_pulse_s=min(widths) if widths else None,
            max_rate_hz=1/min(intervals) if intervals else None,
            gaps_over_segment_s=[[a,b] for a,b in zip(rises,rises[1:]) if b-a>.005])
    return dict(motors=motors,first_step_s=min(first) if first else None,
                last_step_s=max(last) if last else None,motion_s=max(last)-min(first) if first else None)


def stream(tg, lines, timeout, *, clock=time.monotonic, probe_interval=.25, probe_kind="stat"):
    """One reader, reserved control slots, and configurable tid-tagged probes.

    Directives start after previous lines are acknowledged. HOLD-RESUME waits
    the first delay, sends !, then waits for HOLD plus the second delay before ~.
    Never call tg.get concurrently with this loop.
    """
    if probe_kind not in ('stat','qr'):raise ValueError('invalid probe kind')
    if probe_interval is not None and (not math.isfinite(probe_interval) or probe_interval<=0):raise ValueError('invalid probe interval')
    tr=tg.transport;free=tg.get('rx');todo=[l.strip() for l in lines if l.strip()]
    queue=deque();i=acked=0;started=clock();next_probe=started
    pending=None;tid=810000;action=None;last_stat=None;idle_since=None;done='timeout'
    rows,probes,statuses,errors,events=[],[],[],[],[]
    sent_rows=[]
    credit_initial=credit_min=credit_last=free
    credit_rows=[]
    fb=getattr(tg,'identity',{}).get('fb',450)
    legacy=449<=float(fb)<450
    velocity=None;zero_since=None;bang_at=None
    while clock()-started<timeout:
        now=clock()
        if (probe_interval is None or probe_kind=="qr") and idle_since is not None and now-idle_since>=.5:
            done='idle';break
        if pending and now-pending[1]>5: errors.append(f'{probe_kind} probe unanswered for 5 s');break
        if action:
            kind,until,delay=action
            if kind=='pause' and now>=until: action=None
            elif kind=='hold' and now>=until:
                tr.write(b'!',drain_timeout=2);events.append([now,'!']);action=('held',None,delay);last_stat=None;zero_since=None;velocity=None;bang_at=now
            elif kind=='held' and ((not legacy and last_stat==tg.dialect.stat('HOLD')) or
                                  (legacy and zero_since is not None and now-zero_since>=.5)):
                events.append([now,'hold confirmed by velocity' if legacy else 'hold confirmed by stat'])
                action=('resume',now+delay,0)
            elif kind=='resume' and now>=until:
                tr.write(b'~',drain_timeout=2);events.append([now,'~']);action=None;zero_since=None;velocity=None
        if probe_interval is not None and now>=next_probe and pending is None:
            tid+=1;sent=clock();tr.write((json.dumps({probe_kind:None,'tid':tid})+'\n').encode(),drain_timeout=2)
            pending=(tid,sent);next_probe=sent+probe_interval
        while action is None and i<len(todo) and free-len(queue)>RESERVE:
            line=todo[i]
            if line.startswith((';HOST-PAUSE',';HOLD-RESUME')):
                if queue: break
                f=line.split();ds=list(map(float,f[1:]))
                if any(not math.isfinite(v) or v<0 for v in ds): raise ValueError('invalid directive delay')
                if f[0]==';HOST-PAUSE' and len(ds)==1: action=('pause',clock()+ds[0],0)
                elif f[0]==';HOLD-RESUME' and len(ds)==2: action=('hold',clock()+ds[0],ds[1])
                else: raise ValueError('invalid directive: '+line)
                events.append([clock(),line]);i+=1;break
            last_stat=None;idle_since=None;velocity=None
            tr.write((line+'\n').encode(),drain_timeout=2);sent=clock()
            queue.append((i,sent,line));sent_rows.append(dict(index=i,sent=sent,line=line));i+=1
        ev=tr.next_event(.005)
        if ev is None: continue
        if ev.kind=='exception': errors.append(ev.raw);break
        if ev.kind=='status':
            statuses.append([ev.t,ev.body])
            stat=ev.body.get('stat') if isinstance(ev.body,dict) else None
            if isinstance(ev.body,dict) and 'vel' in ev.body:
                velocity=ev.body['vel']
                if action and action[0]=='held' and bang_at is not None and ev.t>=bang_at:
                    if velocity==0:
                        if zero_since is None:zero_since=now
                    else:zero_since=None
                if legacy and velocity!=0:idle_since=None
            if isinstance(stat,int):
                last_stat=stat
                if stat in tg.dialect.stats('ALARM','SHUTDOWN','PANIC'):
                    errors.append(f'terminal machine state {stat}');break
                if probe_interval is None or probe_kind=="qr":
                    if i==len(todo) and not queue and action is None and stat in tg.dialect.idle_stats and (not legacy or velocity==0):
                        if idle_since is None:idle_since=now
                    else:idle_since=None
            continue
        if ev.kind!='response': continue
        footer=_footer(ev);body=ev.body if isinstance(ev.body,dict) else {}
        echo=ev.data.get('tid',body.get('tid'))
        if footer is None: errors.append('response without valid footer');break
        credit_last=footer[1];credit_min=min(credit_min,credit_last)
        credit_rows.append(dict(t=ev.t,free=credit_last,probe=pending is not None and echo==pending[0]))
        if pending and echo==pending[0]:
            value=body.get(probe_kind)
            if footer[0] or type(value) is not int or (probe_kind=='qr' and value<0): errors.append(f'invalid {probe_kind} probe reply');break
            # A priority reply can be the only response after fragmented RX headers
            # become reusable. Refresh credit without acknowledging queued G-code.
            free=footer[1]
            probes.append(dict(sent=pending[1],received=ev.t,rtt_s=ev.t-pending[1],**{probe_kind:value},
                               lines_pending=len(queue),lines_unsent=len(todo)-i))
            pending=None
            if probe_kind=='qr':continue # qr is queue space, never a machine state
            stat=value;last_stat=stat
            if stat in tg.dialect.stats('ALARM','SHUTDOWN','PANIC'):
                errors.append(f'terminal machine state {stat}');break
            if i==len(todo) and not queue and action is None and stat in tg.dialect.idle_stats and (not legacy or velocity==0):
                if idle_since is None: idle_since=now
                if now-idle_since>=.5: done='idle';break
            else: idle_since=None
            continue
        if echo is not None or not queue: errors.append('unmatched response: '+ev.raw);break
        index,sent,line=queue.popleft();acked+=1;free=footer[1]
        rows.append(dict(index=index,sent=sent,acked=ev.t))
        if footer[0]: errors.append(dict(index=index,status=footer[0],line=line));break
    span=rows[-1]['acked']-rows[0]['sent'] if rows else 0
    rtts=[p['rtt_s'] for p in probes if p['lines_pending'] or p['lines_unsent']]
    return dict(completion=done,errors=errors,acked=acked,sent=len(sent_rows),sent_rows=sent_rows,
                pending=len(queue),unsent=sum(not l.startswith((';HOST-PAUSE',';HOLD-RESUME')) for l in todo[i:]),started=started,finished=clock(),ack_rows=rows,
                hold_detection='449 post-! vel=0 plus 0.5 s settle' if legacy else 'HOLD stat',
                probe_mode='off' if probe_interval is None else f'{probe_interval*1000:g}ms',
                probe_kind=probe_kind,probe_ms=None if probe_interval is None else probe_interval*1000,
                rx_credit_initial=credit_initial,rx_credit_min=credit_min,rx_credit_last=credit_last,rx_credit_rows=credit_rows,
                ack_span_s=span,lines_s=acked/span if span>0 else None,probes=probes,
                probe_median_s=percentile(rtts,.5),probe_p95_s=percentile(rtts,.95),
                stat_median_s=percentile(rtts,.5) if probe_kind=='stat' else None,stat_p95_s=percentile(rtts,.95) if probe_kind=='stat' else None,statuses=statuses,events=events)


def write_record(path, record):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(record,indent=2,allow_nan=False)+'\n')


def settings_snapshot(tg, *, include_new=False):
    """Comparable persisted settings, without querying post-449 rt/hold additions."""
    from .snapshot import IDENTITY, INTERNAL
    from .driver import StatusError
    tg.send_gcode('G21',check=True);tg.wait_idle()
    values,missing={},[]
    for token in tg.dialect.persisted_tokens():
        if token in IDENTITY or INTERNAL.match(token) or token == '_hld':
            continue
        if not include_new and (token.startswith('rt') or token in ('spph','spde','coph')):
            continue
        try:values[token]=tg.get(token)
        except StatusError as e:
            if e.response.status!=tg.dialect.status('UNRECOGNIZED_NAME'):raise
            missing.append(token)
    sr=tg.get('sr')
    return dict(values=values,missing=missing,sr=list(sr) if isinstance(sr,dict) else sr)


def validate_firmware(identity,flash,tag,env):
    """Check the board build separately from the host's last-flashed image record.

    known means an image hash was recorded, not an independent device readback.
    CI artifacts may have a known hash but no source checkout/commit metadata.
    """
    import re
    expected_fb=float(env.get('PERF_EXPECT_FB_A','449.01')) if tag=='A' else float(env.get('PERF_EXPECT_FB','450.04'))
    if tag not in ('A','B') or not math.isfinite(expected_fb):raise ValueError('invalid firmware expectation')
    flash=flash_metadata(flash)
    expected_hex=env.get('PERF_EXPECT_HEX') or None
    if expected_hex:
        expected_hex=expected_hex.lower()
        if not re.fullmatch('[0-9a-f]{64}',expected_hex):raise ValueError('PERF_EXPECT_HEX must be a full hex sha256')
    try:actual=float(identity['fb'])
    except (KeyError,ValueError,TypeError):raise ValueError('board fb missing or invalid')
    if not math.isfinite(actual) or abs(actual-expected_fb)>.0005:
        raise ValueError(f'tag {tag} requires fb {expected_fb:.2f}; board reports {actual}')
    if expected_hex and (not flash['known'] or flash['hex_sha256']!=expected_hex):
        raise ValueError('flash record hex sha256 does not match PERF_EXPECT_HEX')
    return dict(expected_fb=expected_fb,expected_hex=expected_hex,flash=flash)


def flash_metadata(flash):
    import re
    flash=dict(flash or {})
    flash.setdefault('commit','')
    sha=flash.get('hex_sha256','')
    flash['known']=bool(re.fullmatch('[0-9a-fA-F]{64}',str(sha)) and not flash.get('legacy') and flash.get('known',True))
    flash['hex_sha256']=str(sha).lower()
    return flash
