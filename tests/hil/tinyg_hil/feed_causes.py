"""Pure assertions shared by hardware cause tests and synthetic negative tests."""
EXACT=4
SINGLE=16
SHORT=8


def exact_stop_lines(lines):
    # TinyG parses G61 as exact path and G61.1 as exact stop.
    return ['G61.1' if line.strip() == 'G61' else line for line in lines]


def check_accounted_cause(fs,index):
    """Completed segment time survives a periodic fl report missing the block.

    This proves participation, not that this was the block's only constraint.
    """
    assert fs and all(s.get('v')==2 and len(s.get('c',[]))==7 for s in fs),'missing fs v2'
    assert max(s['c'][index] for s in fs)>0,f'cause c[{index}] never accrued'


def check_cause(fl,fs,tag,index,*,alone=False):
    check_accounted_cause(fs,index)
    matches=[e for e in fl if e.get('k')==1 and e.get('m',0)&tag]
    assert matches,f'tag {tag} never observed on a feed move'
    if alone: assert any(e['m']==tag for e in matches),'single segment only observed with other constraints'


def check_gaps(fl,fg,resumed):
    assert any(e.get('g')==2 and e.get('k')==0 for e in fl),'no-input gap not observed'
    assert fg and any(g['g'][0]>0 and g['g'][1]>0 for g in fg),'both pending/no-input times required'
    assert any(e.get('k')==1 and 'g' not in e for e in resumed),'gap field did not clear on resumed feed'


def check_command(fs):
    assert fs and any(s.get('v')==2 and len(s.get('c',[]))==7 and s['c'][6]>0 for s in fs),'no v2 command time'
