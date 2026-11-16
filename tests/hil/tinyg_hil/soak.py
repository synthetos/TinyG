"""Opt-in program-stop handling for unloaded-file diagnostics."""
import threading
import time


def mode_for(dialect, requested=None):
    mode=requested or ('line' if 'rxm' in dialect.tokens else 'count')
    if 'rxm' not in dialect.tokens and mode in ('line','g2'):
        raise ValueError('this dialect has no line mode; use SOAK_MODE=count, rts or xon')
    return mode


def resume_policy(value):
    if value not in ('', 'resume'):
        raise ValueError('SOAK_M0 must be unset or resume')
    return value=='resume'


class ProgramStopResumer:
    """Only explicit PROGRAM_STOP observations can authorize a cycle start.

    A fresh repeated stop may need another start (adjacent M0/M1 commands).
    Reports received before the previous start cannot authorize another one.
    This never changes the sender's progress/deadline/stall accounting.
    """
    def __init__(self,tg,lock=None,settle=.5):
        self.tg=tg
        self.lock=lock or threading.Lock()
        self.settle=settle
        self.started=time.monotonic()
        self.last_resume=self.started
        self.events=[]

    def begin(self):
        self.started=self.last_resume=time.monotonic()

    def __call__(self,t,sr):
        stop=self.tg.dialect.stat('PROGRAM_STOP')
        state=sr.get('stat')
        if type(state) is not int or state!=stop or t<=self.last_resume:
            return False
        with self.lock:  # do not interleave with the diagnostic's random !/~ pairs
            if t<=self.last_resume:return False
            time.sleep(self.settle)
            # The transport reader keeps receiving while this callback settles.
            # Do not resume if a newer report says HOLD/RUN/alarm/etc.
            latest=stop
            for ev in reversed(self.tg.transport.history):
                if ev.t<t:break
                if ev.kind=='status' and isinstance(ev.body,dict) and 'stat' in ev.body:
                    if getattr(ev,'damaged',False):return False
                    latest=ev.body['stat'];break
            if latest!=stop:return False
            self.last_resume=time.monotonic()
            self.tg.transport.write(b'~')
            self.events.append(dict(observed=t,resumed=self.last_resume,stat=stop))
            return True


class CapStopTimeout(AssertionError):
    pass


def fold_status(events, until):
    """Latest known status-report values from events received at or before `until` (reports are sent on change)."""
    sr = {}
    for ev in events:
        if ev.t <= until and ev.kind == "status" and isinstance(ev.data, dict) and isinstance(ev.data.get("sr"), dict):
            sr.update(ev.data["sr"])
    return sr


def confirm_hold(next_event, seed, t_req, hold_stat, clock, settle, timeout=15.0):
    """After a feedhold request written at t_req, wait for proof that the hold completed; return the state.

    Evidence is positive only: a report received after t_req completes stat == hold_stat with vel 0, and the folded
    state then stays held for `settle` seconds (the caller passes two status intervals, with the interval enforced
    beforehand). Reports arrive in the order the board sent them, so an older hold report that was followed by a
    resume (sent before the request but received after it) is overtaken by that resume report within the window,
    which cancels the candidate. Reports already folded into `seed` (t <= t_req) are ignored when they come off the
    queue. Silence alone never succeeds: no fresh hold report means CapStopTimeout.
    """
    state, since = dict(seed), None
    while True:
        now = clock()
        if since is not None and now - since >= settle:
            return state
        if now - t_req > timeout:
            raise CapStopTimeout(f"no completed feedhold confirmed within {timeout} s after the request: {state}")
        ev = next_event(0.05)
        if ev is None or ev.t <= t_req or ev.kind != "status" or not isinstance(ev.data, dict):
            continue
        if isinstance(ev.data.get("sr"), dict):
            state.update(ev.data["sr"])
            held = state.get("stat") == hold_stat and state.get("vel", 1) == 0
            since = (since if since is not None else ev.t) if held else None
