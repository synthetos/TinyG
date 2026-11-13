"""Qualify debugger switch stimuli against the original accepted closure."""


def qualify(before, closed, released, lockout_ms):
    start = closed['start']
    elapsed = (released['ticks'] - start) & 0xffffffff
    result = dict(original_start=start, elapsed_ms=elapsed, lockout_ms=lockout_ms)
    def outcome(kind, reason):
        return result | dict(outcome=kind, reason=reason)
    if before['state'] != 0 or before['raw_pin'] != 1 or before['start'] != 0:
        return outcome('invalid', 'input or previous lockout was not clear before closure')
    if closed['raw_pin'] != 0:
        return outcome('invalid', 'closed stimulus was not observed on the pin')
    if closed['state'] != 1:
        return outcome('failure', 'active input was not cached after an applied closure with no prior lockout')
    if not start or closed['delay'] != lockout_ms:
        return outcome('invalid', 'original closure timer was not captured before expiry')
    if released['raw_pin'] != 1:
        return outcome('invalid', 'released stimulus was not observed on the pin')
    if elapsed > lockout_ms:
        return outcome('invalid', 'release was not observed inside the original lockout window')
    # Only now can state/timer assertions mean firmware failures: the observed
    # release is proven to precede the ORIGINAL deadline, including across wrap.
    if released['state'] != 1:
        return outcome('failure', 'cached active state changed inside the original lockout')
    if released['start'] != start or released['delay'] != lockout_ms:
        return outcome('failure', 'timer changed inside the original lockout')
    return outcome('valid', 'released pin observed within the original closure lockout')


def trials(sample, close, release, prepare, record, lockout_ms, attempts=5):
    """Return a qualified trial or None; never retry a proven logic failure."""
    observations = []
    for attempt in range(1, attempts + 1):
        prepare()
        before = sample()
        close()
        closed = sample()
        release()
        released = sample()
        result = qualify(before, closed, released, lockout_ms)
        result.update(attempt=attempt, before=before, closed=closed, released=released)
        observations.append(result)
        record('stimulus_trials', list(observations))
        if result['outcome'] == 'failure':
            raise AssertionError(result['reason'] + ': ' + str(result))
        if result['outcome'] == 'valid':
            return result
    return None
