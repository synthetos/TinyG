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
