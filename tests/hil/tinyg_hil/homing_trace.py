"""Summarize externally captured homing input and step/direction transitions.

The first search direction is negative. This establishes sign without assuming
motor polarity. The switch edges are electrical timestamps, not internal state
transitions; no claim about the exact FEEDHOLD_HOLD instant is made.
"""
from .saleae import rising, level_at


def summarize(edges, initial, steps_per_mm, switch=0, step=6, direction=7):
    pulses=rising(edges[step])
    if not pulses:
        raise ValueError('no captured steps')
    negative=level_at(initial[direction],edges[direction],pulses[0])
    signed=[(t,-1 if level_at(initial[direction],edges[direction],t)==negative else 1) for t in pulses]
    position=0
    positions=[]
    changes=[]
    previous=None
    for t,sign in signed:
        position+=sign/steps_per_mm
        positions.append(position)
        if previous is not None and sign!=previous:changes.append(t)
        previous=sign
    closures=[t for t,v in edges[switch] if v==0]
    openings=[t for t,v in edges[switch] if v==1]
    before_reverse=[t for t,s in signed if s<0 and (not changes or t<changes[0])]
    last_search=max(before_reverse)
    return {'sample_method':'rising steps with sampled direction; electrical input edges',
            'switch_initial':initial[switch],'switch_closures_s':closures,'switch_openings_s':openings,
            'direction_changes_at_steps_s':changes,'first_step_s':pulses[0],'last_step_s':pulses[-1],
            'last_search_step_s':last_search,'step_count':len(pulses),
            'input_to_last_search_step_s':last_search-closures[0] if closures else None,
            'input_to_first_reverse_step_s':changes[0]-closures[0] if closures and changes else None,
            'min_position_mm':min(positions),'max_position_mm':max(positions),
            'final_position_mm':positions[-1],
            'limitation':'No direct observation of internal hold state; repeated success does not exclude a rare race.'}
