"""Promote instrument captures and reproducible plots without opening a board."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil

from .motion_profile import analyze

WINDOWS = {.02:.08, .05:.08, .1:.08, .25:.12, .5:.12, 1.:.16, 2.:.24, 5.:.24, 10.:.24}


def _json(path):
    return json.loads(Path(path).read_text())


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
    fig.savefig(destination,dpi=120);plt.close(fig)
    return data


def _manifest(directory):
    return [{'file':str(p.relative_to(directory)), 'bytes':p.stat().st_size,
             'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in sorted(directory.rglob('*')) if p.is_file() and p.name!='summary.json']


def promote(run, finding, label, dest_root=None):
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
            shutil.copy2(capture,destination/capture.name)
            edges=_json(edge);_write(destination/edge.name,edges)
            data=_plot_case(edges,analysis,destination/f'{stem}.png')
            compact={k:v for k,v in analysis.items() if k not in ('raw','windows','capture','edge_file')}
            compact['capture']=capture.name;compact['edge_file']=edge.name
            compact['windows']=[{k:v for k,v in w.items() if k!='estimates'} for w in data['windows']]
            compact['trace_regeneration']='Recompute from stored edges with tinyg_hil.motion_profile.analyze; raw derivative traces are not duplicated.'
            _write(destination/original.name,compact)
            w=data['windows'][0] if data['windows'] else None
            results.append({'length_mm':analysis['length_mm'],'window_s':w['width_s'] if w else None,
                            'peak_jerk_ratio':w['peak_j_mm_s3']/analysis['commanded_jerk_mm_s3'] if w and w['resolved'] else None,
                            'pulse_span_s':data['first_to_last_step_s'],'plot':f'{stem}.png','analysis':original.name})
        for name in ('profile-calibration.json','baseline-fixed-comparison.json'):
            if (run/name).exists():shutil.copy2(run/name,destination/name)
        artifacts=_manifest(destination)
        summary={'finding':finding,'label':label,'result':'measurement; gate verdict requires review',
                 'firmware':firmware,'harness_commit':record.get('commit'),'bench':record.get('identity'),
                 'run_record':str(run/'run.json'),'source_artifacts':source_artifacts,
                 'method':{'sample_rate':12_000_000,'channels':{'6':'M1 step','7':'M1 direction'},'fit':'degree-5 local position polynomial; >=12 edges, >=80% window coverage','window_rule':'baseline-selected by move length, unchanged for after','raw':'adjacent step intervals then finite differences','time':'first-to-last step, not command-to-rest'},
                 'limits_and_tolerance':'Configured J comes from xjm; no unconditional pass inferred. Empirical phase calibration is recorded separately. Conservative step-quantization bound does not include model bias. .01 mm unresolved; .02 mm biased low. No mechanical jerk claim.',
                 'cases':sorted(results,key=lambda x:x['length_mm']),'artifacts':artifacts,
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
        length=case['length_mm'];stem=f'profile-X-{length:g}mm'
        if not (before/f'{stem}-analysis.json').exists():continue
        fig,axes=plt.subplots(3,1,figsize=(8,7),sharex=True,layout='constrained')
        for label,root,color in [('Before',before,'#b45e15'),('After',after,'#087c89')]:
            info=_json(root/f'{stem}-analysis.json');edges=_json(root/f'{stem}-edges.json');start,width,data=_series(edges,info)
            estimates=data['windows'][0]['estimates'] if width else []
            for ax,key in zip(axes,('v','a','j')):ax.plot([e['t']-start for e in estimates],[e[key] for e in estimates],label=label,color=color,lw=1)
        for sign in (-1,1):axes[2].axhline(sign*info['commanded_jerk_mm_s3'],color='#bd342d',ls='--',lw=.8)
        for ax,label in zip(axes,('Velocity (mm/s)','Acceleration (mm/s²)','Jerk (mm/s³)')):ax.set_ylabel(label);ax.grid(alpha=.2)
        axes[0].legend();axes[-1].set_xlabel('Time from first step (s; initial step phase differs)')
        fig.suptitle(f'X {length:g} mm · before/after step envelope\n'+(f'Same {width*1000:g} ms fit window; red = configured jerk' if width else 'Unresolved: insufficient steps'))
        name=f'{stem}-overlay.png';fig.savefig(after/name,dpi=120);plt.close(fig);pairs.append(name)
    a['overlay_before']=str(before);a['overlays']=pairs;a['artifacts']=_manifest(after)
    a['large_artifacts']=[x['file'] for x in a['artifacts'] if x['bytes']>1_000_000]
    _write(after/'summary.json',a)
    return pairs


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);subs=parser.add_subparsers(dest='action',required=True)
    p=subs.add_parser('promote');p.add_argument('run');p.add_argument('--finding',required=True);p.add_argument('--label',required=True,choices=['before','after']);p.add_argument('--dest-root');p.add_argument('--before',help='promoted before directory for overlays')
    args=parser.parse_args(argv);destination,summary=promote(args.run,args.finding,args.label,args.dest_root)
    if args.before:overlay(args.before,destination);summary=_json(destination/'summary.json')
    print(destination)
    for name in summary['large_artifacts']:print(f'WARNING artifact exceeds 1 MB: {name}')
    return 0


if __name__=='__main__':raise SystemExit(main())
