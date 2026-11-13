"""E060 measurements on capture time; host command alignment has bounded uncertainty."""


def measure(edges,pwm,commands,started_wall,*,flush=False,clock_uncertainty_s=.1,pwm_window_s=.02):
    rises=sorted({t for name,trace in edges.items() if name.endswith(' step') for t,v in trace if v==1})
    bang=commands['!']-started_wall
    release=commands['%' if flush else '~']-started_wall
    before=[t for t in rises if t<release-clock_uncertainty_s]
    after=[t for t in rises if t>release+clock_uncertainty_s]
    if not before: raise ValueError('no motion before release')
    stop=before[-1]
    if stop<bang-clock_uncertainty_s: raise ValueError('motion stopped before feedhold')
    # Diagnostic sets PWM on to 80% and off to 10%; a full 20ms window resolves the 100Hz carrier.
    off=[w['t'] for w in pwm if bang-clock_uncertainty_s<=w['t']<release and w['duty']<.45]
    on=[w['t'] for w in pwm if w['t']>=release-clock_uncertainty_s and w['duty']>.45]
    result=dict(stop_s=stop,command_clock_uncertainty_s=clock_uncertainty_s,pwm_window_s=pwm_window_s,
                hold_to_stop_s=stop-bang,stop_to_spindle_off_s=off[0]-stop if off else None,
                resume_to_spindle_on_s=on[0]-release if on and not flush else None,
                spindle_on_to_motion_s=after[0]-on[0] if after and on and not flush else None)
    if flush and after: raise ValueError('motion restarted after flush')
    if not flush and not after: raise ValueError('no resumed motion captured')
    observed=[w for w in pwm if bang-clock_uncertainty_s<=w['t']<=release+.5]
    if not observed: raise ValueError('no PWM evidence across hold and release')
    if any(w['duty']<.45 for w in observed): raise ValueError('spindle changed during motion-only hold')
    result['stop_to_spindle_off_s']=None
    result['resume_to_spindle_on_s']=None
    result['spindle_on_to_motion_s']=None
    result['resume_to_motion_s']=after[0]-release if after and not flush else None
    result['spindle_unchanged']=True
    return result
