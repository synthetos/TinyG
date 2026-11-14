"""Decode the separate N111 main-loop planning timer sites."""
FIELDS = ('calls', 'overflows', 'maximum_ticks', 'site', 'section_at_maximum')


def decode_sample(values):
    row = {name: int(values['motion_profile.'+name]) for name in FIELDS}
    if row['site'] not in (6, 7):
        raise ValueError('requires MOTION_PROFILE=6 or 7 and the matching ELF')
    if row['overflows'] or row['maximum_ticks'] == 65535:
        raise ValueError('planning timer overflow: no valid worst-case bound')
    if row['calls'] <= 0 or row['maximum_ticks'] <= 0:
        raise ValueError('no planning samples')
    if not 0 <= row['section_at_maximum'] <= 32:
        raise ValueError('invalid queue occupancy at maximum')
    row.update(tick_us=2., maximum_us=2.*row['maximum_ticks'],
               queued_at_maximum=row.pop('section_at_maximum'),
               scope='lookahead_pass' if row['site'] == 6 else 'one_trapezoid')
    return row
