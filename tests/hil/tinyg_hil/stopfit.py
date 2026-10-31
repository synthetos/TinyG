"""Fit a feedhold/fast-stop deceleration in a step capture with the firmware's own segmented profile.

What plan_exec.c executes for a stop from cruise v (_exec_aline_tail, _init_forward_diffs):
  tail time T = 2 L / v with L = v sqrt(v / J) / sqrt(ratio)    (mp_get_target_length; E031 fast ratio)
  segments n = ceil(T / NOM_SEGMENT_USEC), each h = T / n long
  segment k runs at the quintic v(s) = v (1 - (10 s^3 - 15 s^4 + 6 s^5)) sampled at its midpoint
  s = (k + 1/2) / n (the forward differences start at h/2), and the DDA turns each segment into steps
  on a FREQUENCY_DDA tick grid, carrying the fraction between segments.
The planned jerk is reported as J_eff = 4 v / T^2, the J for which T = 2 sqrt(v / J), so a normal hold
is 1 x xjm and an E031 fast stop is ratio x xjm. The delay is trip edge to the start of the first tail
segment (the tail starts on a segment boundary of the running move).

synthesize() builds step times for a known delay and T; validate() measures the estimator's bias on
such traces, so a fitted delay or jerk carries a stated error.
"""
import bisect
import math

NOM_SEGMENT_S = 0.005          # NOM_SEGMENT_USEC
FREQUENCY_DDA = 50000.0        # hardware.h


def tail_segments(v, T, nominal=NOM_SEGMENT_S):
    """[(segment seconds, segment velocity), ...] for a stop from v lasting T."""
    n = max(1, math.ceil(T / nominal - 1e-9))
    h = T / n
    out = []
    for k in range(n):
        s = (k + 0.5) / n
        out.append((h, v * (1 - (10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5))))
    return out


class _Tail:
    """Cumulative position of the segmented tail, for fast evaluation at many times."""

    def __init__(self, v, T):
        self.v, self.T = v, T
        self.starts, self.xs, self.vs = [], [], []
        clock = x = 0.0
        for h, vk in tail_segments(v, T):
            self.starts.append(clock); self.xs.append(x); self.vs.append(vk)
            clock, x = clock + h, x + vk * h
        self.length = x

    def at(self, u):
        """Distance u seconds into the tail."""
        if u <= 0:
            return 0.0
        if u >= self.T:
            return self.length
        k = bisect.bisect_right(self.starts, u) - 1
        return self.xs[k] + self.vs[k] * (u - self.starts[k])


def position(t, v, delay, T, tail=None):
    """Ideal segmented position (no DDA quantization) t seconds after the trip."""
    tail = tail or _Tail(v, T)
    return v * min(t, delay) + tail.at(t - delay)


def synthesize(v, delay, T, spm, phase=0.0, dda=FREQUENCY_DDA):
    """Step times after the trip for a known delay and T, quantized to the DDA tick grid.

    phase (0..1) shifts the tick grid and the starting step fraction relative to the trip.
    """
    tail, tick = _Tail(v, T), 1.0 / dda
    times, acc = [], phase % 1.0
    t = (phase % 1.0) * tick
    end = delay + T + 2 * tick
    last = position(t, v, delay, T, tail) * spm
    while t < end:
        t += tick
        now = position(t, v, delay, T, tail) * spm
        acc += now - last
        last = now
        while acc >= 1.0:
            acc -= 1.0
            times.append(t)
    return times


def _sse(points, v, d, T):
    tail = _Tail(v, T)
    return sum((x - v * min(t, d) - tail.at(t - d)) ** 2 for t, x in points)


def fit(times, v_guess, spm, v_span=0.01, v_step=0.0005, max_delay=0.060):
    """Least-squares (v mm/s, delay s, T s, rms mm) for step times measured from the trip edge.

    The segmented quintic is symmetric about its midpoint, so a tail covers exactly v T / 2 and the
    run-out after the trip is v d + v T / 2: T follows from the measured run-out for each (v, d). The
    delay trades against v through that constraint, so the cruise velocity is fitted too, within
    v_span (fraction) of v_guess, rather than taken from a step-rate estimate.
    """
    if not times:
        raise ValueError("no steps after the trip")
    total = len(times) / spm
    stride = max(1, len(times) // 600)
    points = [(t, (i + 1) / spm) for i, t in enumerate(times)][::stride]
    vs = [v_guess * (1 + k * v_step) for k in range(-int(v_span / v_step), int(v_span / v_step) + 1)]

    def search(v_list, d_list):
        best = None
        for v in v_list:
            for d in d_list:
                T = 2 * (total / v - d)
                if T <= 0:
                    continue
                e = _sse(points, v, d, T)
                if best is None or e < best[0]:
                    best = (e, v, d, T)
        return best
    best = search(vs, [i * 0.00025 for i in range(int(max_delay / 0.00025) + 1)])
    k = vs.index(best[1])
    best = search(vs[max(0, k - 2):k + 3], [max(0.0, best[2] + i * 0.00005) for i in range(-10, 11)])
    return best[1], best[2], best[3], math.sqrt(best[0] / len(points))


def jerk_eff(v, T):
    return 4 * v / T ** 2


def validate(v, spm, xjm_jerk, ratios=(1.0, 9.0), delays=(0.0035, 0.0075, 0.0125, 0.0175, 0.0350),
             phases=(0.0, 0.37, 0.71), v_errors=(-0.004, 0.004)):
    """Estimator error on synthetic firmware-profile traces, started from a wrong cruise guess:
    max |fitted - true| for delay (s), and max relative error for jerk ratio and cruise velocity."""
    worst_d = worst_j = worst_v = 0.0
    cases = []
    for r in ratios:
        T = 2 * math.sqrt(v / (xjm_jerk * r))
        for d in delays:
            for i, p in enumerate(phases):
                guess = v * (1 + v_errors[i % len(v_errors)])
                fv, fd, fT, rms = fit(synthesize(v, d, T, spm, p), guess, spm)
                fj = jerk_eff(fv, fT) / xjm_jerk
                worst_d, worst_j, worst_v = max(worst_d, abs(fd - d)), max(worst_j, abs(fj - r) / r), max(worst_v, abs(fv - v) / v)
                cases.append({"ratio": r, "delay": d, "phase": p, "v_guess": guess, "fit_v": fv, "fit_delay": fd,
                              "fit_ratio": fj, "rms_mm": rms})
    return {"max_delay_error_s": worst_d, "max_jerk_rel_error": worst_j, "max_v_rel_error": worst_v, "cases": cases}
