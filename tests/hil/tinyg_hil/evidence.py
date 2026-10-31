"""Promote instrument captures and reproducible plots without opening a board."""
import argparse
import hashlib
import gzip
import json
from pathlib import Path
import re
import shutil
import subprocess

from .motion_profile import analyze

WINDOWS = {.02:.08, .05:.08, .1:.08, .25:.12, .5:.12, 1.:.16, 2.:.24, 5.:.24, 10.:.24}


def _json(path):
    path=Path(path)
    return json.loads(gzip.decompress(path.read_bytes()) if path.suffix=='.gz' else path.read_text())


def _write(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


def _plot_module():
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        return plt
    except ImportError as exc:
        raise RuntimeError('Evidence plotting needs matplotlib; install tinyg-hil[evidence] or matplotlib>=3.8') from exc


def _series(edges, analysis):
    rises=edges['step_rises']
    width=WINDOWS.get(analysis['length_mm'])
    data=analyze(rises,analysis['steps_per_mm'],sample_rate=edges['sample_rate'],windows=(width,) if width else ())
    return rises[0],width,data


def _plot_case(edges, analysis, destination):
    plt=_plot_module();start,width,data=_series(edges,analysis)
    fig,axes=plt.subplots(3,2,figsize=(10,7),sharex=True,layout='constrained')
    labels=[('velocity','v','Velocity (mm/s)'),('acceleration','a','Acceleration (mm/s²)'),('jerk','j','Jerk (mm/s³)')]
    estimates=data['windows'][0]['estimates'] if width else []
    for row,(raw,key,label) in enumerate(labels):
        pairs=data['raw'][raw];stride=max(1,len(pairs)//1600);pairs=pairs[::stride]
        axes[row,0].plot([t-start for t,_ in pairs],[v for _,v in pairs],color='#536b85',lw=.65)
        axes[row,1].plot([e['t']-start for e in estimates],[e[key] for e in estimates],color='#087c89',lw=1.2)
        axes[row,0].set_ylabel(label)
        if row in (1,2):axes[row,0].set_yscale('symlog',linthresh=1)
        for ax in axes[row]:ax.grid(alpha=.2)
        if row==2:
            for ax in axes[row]:
                for sign in (-1,1):ax.axhline(sign*analysis['commanded_jerk_mm_s3'],color='#bd342d',lw=.8,ls='--')
    axes[0,0].set_title('Raw interval derivatives (decimated for display)')
    axes[0,1].set_title(f'Position fit: {width*1000:g} ms' if width else 'Unresolved: too few steps')
    for ax in axes[-1]:ax.set_xlabel('Time from first step (s)')
    fig.suptitle(f"X {analysis['length_mm']:g} mm · F{analysis['feed_mm_min']:g} · J {analysis['commanded_jerk_mm_s3']:.3f} mm/s³\nStep-command envelope; red = configured jerk, not a mechanical measurement",fontsize=11)
    fig.savefig(destination,dpi=90);plt.close(fig)
    return data


def _manifest(directory):
    return [{'file':str(p.relative_to(directory)), 'bytes':p.stat().st_size,
             'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in sorted(directory.rglob('*')) if p.is_file() and p.name!='summary.json']


def promote(run, finding, label, dest_root=None, representative=(.05,1.,10.)):
    if not re.fullmatch(r'E\d{3}',finding):raise ValueError('finding must look like E015')
    if label not in ('before','after'):raise ValueError('label must be before or after')
    run=Path(run).resolve();record=_json(run/'run.json');firmware=record.get('firmware',{})
    commit=firmware.get('commit','')
    if not firmware.get('known') or not re.fullmatch(r'[0-9a-f]{7,40}',commit):raise ValueError('record needs known firmware commit')
    if firmware.get('dirty'):raise ValueError('refuse to promote an unidentified dirty firmware build')
    root=Path(dest_root) if dest_root else Path(__file__).resolve().parents[1]/'evidence'
    destination=root/finding/f'{commit}-{label}'
    if destination.exists():raise FileExistsError(destination)
    cases=sorted(run.glob('profile-X-*-analysis.json'))
    if not cases:raise ValueError('run has no supported motion-profile artifacts')
    _plot_module()  # Fail before creating an incomplete store if plotting is unavailable.
    destination.mkdir(parents=True)
    shutil.copy2(run/'run.json',destination/'source-run.json')
    source_artifacts=[];results=[]
    try:
        for original in cases:
            analysis=_json(original);stem=original.name.removesuffix('-analysis.json')
            edge=run/f'{stem}-edges.json';capture=run/f'{stem}.sal'
            for p in (original,edge,capture):
                source_artifacts.append({'file':p.name,'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'bytes':p.stat().st_size})
            selected=analysis['length_mm'] in representative
            edges=_json(edge)
            if selected:
                shutil.copy2(capture,destination/capture.name)
                (destination/(edge.name+'.gz')).write_bytes(gzip.compress(json.dumps(edges,separators=(',',':')).encode(),mtime=0))
                data=_plot_case(edges,analysis,destination/f'{stem}.png')
            else:
                _,_,data=_series(edges,analysis)
            compact={k:v for k,v in analysis.items() if k not in ('raw','windows','capture','edge_file')}
            compact['capture']=capture.name if selected else None;compact['edge_file']=edge.name+'.gz' if selected else None
            compact['source_capture']=str(capture);compact['source_edges']=str(edge)
            compact['windows']=[{k:v for k,v in w.items() if k!='estimates'} for w in data['windows']]
            compact['trace_regeneration']='Recompute from representative stored gzip edges or the full source run with tinyg_hil.motion_profile.analyze; derivative traces are not duplicated.'
            _write(destination/original.name,compact)
            w=data['windows'][0] if data['windows'] else None
            results.append({'length_mm':analysis['length_mm'],'window_s':w['width_s'] if w else None,
                            'peak_jerk_ratio':w['peak_j_mm_s3']/analysis['commanded_jerk_mm_s3'] if w and w['resolved'] else None,
                            'pulse_span_s':data['first_to_last_step_s'],'plot':f'{stem}.png' if selected else None,'analysis':original.name})
        for name in ('profile-calibration.json','baseline-fixed-comparison.json'):
            if (run/name).exists():shutil.copy2(run/name,destination/name)
        artifacts=_manifest(destination)
        revision=subprocess.run(['git','-C',str(Path(__file__).resolve().parents[3]),'rev-parse','HEAD'],capture_output=True,text=True)
        dirty=subprocess.run(['git','-C',str(Path(__file__).resolve().parents[3]),'diff','--quiet','HEAD','--',str(Path(__file__).resolve())]).returncode
        summary={'promoter_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'promoter_commit':revision.stdout.strip() if revision.returncode==0 and dirty==0 else None,'finding':finding,'label':label,'result':'measurement; gate verdict requires review',
                 'firmware':firmware,'harness_commit':record.get('commit'),'bench':record.get('identity'),
                 'run_record':str(run/'run.json'),'source_artifacts':source_artifacts,
                 'method':{'sample_rate':12_000_000,'channels':{'6':'M1 step','7':'M1 direction'},'fit':'degree-5 local position polynomial; >=12 edges, >=80% window coverage','window_rule':'baseline-selected by move length, unchanged for after','raw':'adjacent step intervals then finite differences','time':'first-to-last step, not command-to-rest'},
                 'limits_and_tolerance':'Configured J comes from xjm; no unconditional pass inferred. Empirical phase calibration is recorded separately. Conservative step-quantization bound does not include model bias. .01 mm unresolved; .02 mm biased low. No mechanical jerk claim.',
                 'representative_lengths_mm':list(representative),'cases':sorted(results,key=lambda x:x['length_mm']),'artifacts':artifacts,
                 'large_artifacts':[a['file'] for a in artifacts if a['bytes']>1_000_000]}
        _write(destination/'summary.json',summary)
    except Exception:
        # A partial directory remains visibly incomplete (no summary) for diagnosis.
        raise
    return destination,summary


def overlay(before,after):
    before,after=Path(before),Path(after);b=_json(before/'summary.json');a=_json(after/'summary.json')
    if b['finding']!=a['finding']:raise ValueError('findings differ')
    plt=_plot_module();pairs=[]
    for case in a['cases']:
        if not case.get('plot'):continue
        length=case['length_mm'];stem=f'profile-X-{length:g}mm'
        if not (before/f'{stem}-analysis.json').exists():continue
        fig,axes=plt.subplots(3,1,figsize=(8,7),sharex=True,layout='constrained')
        for label,root,color in [('Before',before,'#b45e15'),('After',after,'#087c89')]:
            info=_json(root/f'{stem}-analysis.json');edges=_json(root/info['edge_file']);start,width,data=_series(edges,info)
            estimates=data['windows'][0]['estimates'] if width else []
            for ax,key in zip(axes,('v','a','j')):ax.plot([e['t']-start for e in estimates],[e[key] for e in estimates],label=label,color=color,lw=1)
        for sign in (-1,1):axes[2].axhline(sign*info['commanded_jerk_mm_s3'],color='#bd342d',ls='--',lw=.8)
        for ax,label in zip(axes,('Velocity (mm/s)','Acceleration (mm/s²)','Jerk (mm/s³)')):ax.set_ylabel(label);ax.grid(alpha=.2)
        axes[0].legend();axes[-1].set_xlabel('Time from first step (s; initial step phase differs)')
        fig.suptitle(f'X {length:g} mm · before/after step envelope\n'+(f'Same {width*1000:g} ms fit window; red = configured jerk' if width else 'Unresolved: insufficient steps'))
        name=f'{stem}-overlay.png';fig.savefig(after/name,dpi=90);plt.close(fig);pairs.append(name)
    a['overlay_before']=str(before);a['overlays']=pairs;a['artifacts']=_manifest(after)
    a['large_artifacts']=[x['file'] for x in a['artifacts'] if x['bytes']>1_000_000]
    _write(after/'summary.json',a)
    return pairs


TIMING_CHANNELS={'step':'M1 step','x_min':'X min'}


def _timing_plot(edges, captures_label, destination, marker=None):
    """Step rate in 5 ms bins from the stored transitions; marker = a channel whose first falling edge is drawn."""
    plt=_plot_module();tr=edges['transitions'];step=[c for c in tr if c!=str(marker)][0] if marker is not None else next(iter(tr))
    rises=[t for t,level in tr[step] if level==1]
    if not rises:return
    t0=rises[0];bins={}
    for t in rises:bins[int((t-t0)/.005)]=bins.get(int((t-t0)/.005),0)+1
    xs=list(range(0,max(bins)+2));fig,ax=plt.subplots(figsize=(9,3.6),layout='constrained')   # empty bins kept: a stop shows as zero
    ax.step([k*.005 for k in xs],[bins.get(k,0)/.005 for k in xs],where='post',color='#087c89',lw=1)
    if marker is not None:
        falls=[t for t,level in tr[str(marker)] if level==0]
        if falls:ax.axvline(falls[0]-t0,color='#bd342d',lw=.9,ls='--',label='switch trip (ch%s low)'%marker);ax.legend()
    ax.set_xlabel('Time from first step (s)');ax.set_ylabel('Steps/s (5 ms bins)');ax.grid(alpha=.2)
    ax.set_title(captures_label,fontsize=10);fig.savefig(destination,dpi=90);plt.close(fig)


def promote_timing(run, finding, label, captures, dest_root=None, marker=None, plot=True, stopfit=None):
    """Promote named timing-diagnostic captures (stems in the run directory) with the test records that cite them.

    stopfit=(step channel, steps/mm, cruise guess mm/s, xjm jerk mm/s^3): also fit each capture's stop after the
    marker channel's first falling edge with tinyg_hil.stopfit and store the result in stopfit.json."""
    if not re.fullmatch(r'E\d{3}',finding):raise ValueError('finding must look like E089')
    if label not in ('before','after'):raise ValueError('label must be before or after')
    run=Path(run).resolve();record=_json(run/'run.json');firmware=record.get('firmware',{})
    commit=firmware.get('commit','')
    if not firmware.get('known') or not re.fullmatch(r'[0-9a-f]{7,40}',commit):raise ValueError('record needs known firmware commit')
    if firmware.get('dirty'):raise ValueError('refuse to promote an unidentified dirty firmware build')
    if plot:_plot_module()
    root=Path(dest_root) if dest_root else Path(__file__).resolve().parents[1]/'evidence'
    destination=root/finding/f'{commit}-{label}'
    if destination.exists():raise FileExistsError(destination)
    destination.mkdir(parents=True)
    shutil.copy2(run/'run.json',destination/'source-run.json')
    tests=sorted((run/'tests').glob('*.json'))
    source=[]
    for t in tests:
        shutil.copy2(t,destination/('test-'+t.name))
    for p in sorted(run.glob('*.sal'))+sorted(run.glob('*-edges.json')):
        source.append({'file':p.name,'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'bytes':p.stat().st_size})
    for stem in captures:
        sal,edge=run/f'{stem}.sal',run/f'{stem}-edges.json'
        shutil.copy2(sal,destination/sal.name);edges=_json(edge)
        (destination/(edge.name+'.gz')).write_bytes(gzip.compress(json.dumps(edges,separators=(',',':')).encode(),mtime=0))
        if plot:_timing_plot(edges,f'{finding} {label} {commit} {stem}',destination/f'{stem}.png',marker)
    if stopfit:
        from . import stopfit as sf
        step,spm,guess,xjm_jerk=stopfit;fits={}
        for stem in captures:
            tr=_json(run/f'{stem}-edges.json')['transitions'];trip=[t for t,level in tr[str(marker)] if level==0][0]
            after=[t-trip for t,level in tr[str(step)] if level==1 and t>=trip]
            v,d,T,rms=sf.fit(after,guess,spm)
            fits[stem]={'fit_cruise_mm_s':v,'fit_delay_s':d,'fit_tail_s':T,'fit_jerk_over_xjm':sf.jerk_eff(v,T)/xjm_jerk,
                        'fit_rms_mm':rms,'overrun_mm':len(after)/spm}
        _write(destination/'stopfit.json',{'model':'tinyg_hil.stopfit segmented quintic tail, fitted cruise',
               'stopfit_source_sha256':hashlib.sha256(Path(sf.__file__).read_bytes()).hexdigest(),
               'estimator_error':'see tests/test_stopfit_offline.py (synthetic firmware-profile traces)','fits':fits})
    artifacts=_manifest(destination)
    revision=subprocess.run(['git','-C',str(Path(__file__).resolve().parents[3]),'rev-parse','HEAD'],capture_output=True,text=True)
    summary={'promoter_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
             'promoter_commit':revision.stdout.strip() if revision.returncode==0 else None,'finding':finding,'label':label,
             'result':'measurement; gate verdict is the diagnostic test outcome in the copied test record, reviewed',
             'firmware':firmware,'harness_commit':record.get('commit'),'bench':record.get('identity'),
             'run_record':str(run/'run.json'),'representative_captures':list(captures),'source_artifacts':source,
             'method':'tinyg_hil.saleae Capture(save_as=...): Saleae Logic 8 digital capture; sample rate and channels in each edges file',
             'artifacts':artifacts,'large_artifacts':[a['file'] for a in artifacts if a['bytes']>1_000_000]}
    _write(destination/'summary.json',summary)
    return destination,summary


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);subs=parser.add_subparsers(dest='action',required=True)
    p=subs.add_parser('promote');p.add_argument('run');p.add_argument('--finding',required=True);p.add_argument('--label',required=True,choices=['before','after']);p.add_argument('--dest-root');p.add_argument('--representative',type=float,nargs='+',default=[.05,1.,10.]);p.add_argument('--before',help='promoted before directory for overlays')
    t=subs.add_parser('promote-timing');t.add_argument('run');t.add_argument('--finding',required=True);t.add_argument('--label',required=True,choices=['before','after']);t.add_argument('--capture',action='append',required=True,help='capture stem in the run, repeatable');t.add_argument('--marker',type=int,help='analyzer channel whose first falling edge is marked');t.add_argument('--dest-root');t.add_argument('--stopfit',type=float,nargs=4,metavar=('STEP_CH','SPM','V','XJM_JERK'))
    args=parser.parse_args(argv)
    if args.action=='promote-timing':
        destination,summary=promote_timing(args.run,args.finding,args.label,args.capture,args.dest_root,args.marker,
                                           stopfit=(int(args.stopfit[0]),*args.stopfit[1:]) if args.stopfit else None)
        print(destination)
        for name in summary['large_artifacts']:print(f'WARNING artifact exceeds 1 MB: {name}')
        return 0
    destination,summary=promote(args.run,args.finding,args.label,args.dest_root,args.representative)
    if args.before:overlay(args.before,destination);summary=_json(destination/'summary.json')
    print(destination)
    for name in summary['large_artifacts']:print(f'WARNING artifact exceeds 1 MB: {name}')
    return 0


if __name__=='__main__':raise SystemExit(main())
