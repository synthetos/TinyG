"""Step-command envelope estimates; these are not mechanical acceleration measurements.

Raw velocity uses consecutive rising-edge intervals. Raw acceleration/jerk are
finite differences on their midpoint times. Smoothed derivatives fit degree-five
position polynomials to edge count / steps-per-mm in fixed centered time windows.
Windows are 10/20/40/80 ms (at least 12 edges and 80% temporal coverage).

The reported quantization bound is the L1 derivative-weight norm times one step
plus peak raw speed times (one 50 kHz DDA tick + one analyzer sample). It bounds
that assumed position/timestamp error, NOT model bias or mechanical jerk. Window
sensitivity and synthetic calibration must also be considered; no automatic
jerk-limit pass is inferred from a low smoothed peak.
"""
import bisect
import math


def difference(times, values):
    return ([(a+b)/2 for a,b in zip(times,times[1:])],
            [(b-a)/(tb-ta) for ta,tb,a,b in zip(times,times[1:],values,values[1:])])


def _inverse(matrix):
    n=len(matrix)
    rows=[list(row)+[float(i==j) for j in range(n)] for i,row in enumerate(matrix)]
    for col in range(n):
        pivot=max(range(col,n),key=lambda i:abs(rows[i][col]))
        if abs(rows[pivot][col])<1e-10:
            return None
        rows[col],rows[pivot]=rows[pivot],rows[col]
        scale=rows[col][col];rows[col]=[x/scale for x in rows[col]]
        for i in range(n):
            if i!=col:
                scale=rows[i][col]
                rows[i]=[a-scale*b for a,b in zip(rows[i],rows[col])]
    return [row[n:] for row in rows]


def analyze(rises, steps_per_mm, sample_rate=12_000_000, windows=(.01,.02,.04,.08)):
    if steps_per_mm<=0 or len(rises)<4 or any(b<=a for a,b in zip(rises,rises[1:])):
        raise ValueError('need positive geometry and at least four strictly ordered edges')
    velocity_t=[(a+b)/2 for a,b in zip(rises,rises[1:])]
    velocity=[1/(b-a)/steps_per_mm for a,b in zip(rises,rises[1:])]
    accel_t,accel=difference(velocity_t,velocity)
    jerk_t,jerk=difference(accel_t,accel)
    raw={'velocity':list(zip(velocity_t,velocity)), 'acceleration':list(zip(accel_t,accel)),
         'jerk':list(zip(jerk_t,jerk))}
    peak=lambda xs:max(map(abs,xs),default=0)
    out={'step_count':len(rises),'first_to_last_step_s':rises[-1]-rises[0],
         'raw_peak_v_mm_s':peak(velocity),'raw_peak_a_mm_s2':peak(accel),
         'raw_peak_j_mm_s3':peak(jerk),'raw':raw,'windows':[],
         'position_error_assumption_mm':1/steps_per_mm+peak(velocity)*(1/50000+1/sample_rate)}
    for width in windows:
        half=width/2;start=rises[0]+half;end=rises[-1]-half
        estimates=[]
        if end>start:
            count=min(160,max(2,int((end-start)/(width/8))+1))
            for c in range(count):
                center=start+(end-start)*c/(count-1)
                lo=bisect.bisect_left(rises,center-half);hi=bisect.bisect_right(rises,center+half)
                if hi-lo<12 or rises[hi-1]-rises[lo]<.8*width:continue
                # Uniformly subsample very dense windows to cap analysis cost.
                ids=list(range(lo,hi,max(1,(hi-lo)//128)))
                powers=[[((rises[i]-center)/half)**k for k in range(6)] for i in ids]
                gram=[[sum(p[a]*p[b] for p in powers) for b in range(6)] for a in range(6)]
                inv=_inverse(gram)
                if inv is None:continue
                y=[(i-lo)/steps_per_mm for i in ids]
                rhs=[sum(p[k]*v for p,v in zip(powers,y)) for k in range(6)]
                coef=[sum(a*b for a,b in zip(row,rhs)) for row in inv]
                weights=[6/half**3*sum(a*b for a,b in zip(inv[3],p)) for p in powers]
                error=sum(map(abs,weights))*out['position_error_assumption_mm']
                estimates.append({'t':center,'v':coef[1]/half,'a':2*coef[2]/half**2,
                                  'j':6*coef[3]/half**3,'j_quantization_bound':error})
        out['windows'].append({'width_s':width,'resolved':bool(estimates),'estimates':estimates,
            'peak_v_mm_s':peak([x['v'] for x in estimates]),
            'peak_a_mm_s2':peak([x['a'] for x in estimates]),
            'peak_j_mm_s3':peak([x['j'] for x in estimates]),
            'peak_j_upper_assumed_mm_s3':max((abs(x['j'])+x['j_quantization_bound'] for x in estimates),default=None)})
    return out


def synthetic_edges(position, duration, steps_per_mm, tick=1/50000, phase=0):
    """Monotonic reference trajectory, sampled by first crossing each step boundary."""
    out=[]
    for step in range(1,math.floor(position(duration)*steps_per_mm)+1):
        lo,hi=0.,duration
        for _ in range(45):
            mid=(lo+hi)/2
            if position(mid)*steps_per_mm<step:lo=mid
            else:hi=mid
        t=(math.ceil((hi+phase)/tick)*tick-phase) if tick else hi
        if not out or t>out[-1]:out.append(t)
    return out
