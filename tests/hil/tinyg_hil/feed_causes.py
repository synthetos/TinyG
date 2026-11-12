"""Pure assertions shared by hardware cause tests and synthetic negative tests."""
EXACT=4
SINGLE=16
SHORT=8


def check_cause(fl,fs,tag,index,*,alone=False):
    assert fs and all(s.get('v')==2 and len(s.get('c',[]))==7 for s in fs),'missing fs v2'
    assert max(s['c'][index] for s in fs)>0,f'cause c[{index}] never accrued'
    matches=[e for e in fl if e.get('k')==1 and e.get('m',0)&tag]
    assert matches,f'tag {tag} never observed on a feed move'
    if alone: assert any(e['m']==tag for e in matches),'single segment only observed with other constraints'


def check_gaps(fl,fg,resumed):
    assert any(e.get('g')==2 and e.get('k')==0 for e in fl),'no-input gap not observed'
    assert fg and any(g['g'][0]>0 and g['g'][1]>0 for g in fg),'both pending/no-input times required'
    assert any(e.get('k')==1 and 'g' not in e for e in resumed),'gap field did not clear on resumed feed'


def check_command(fs):
    assert fs and any(s.get('v')==2 and len(s.get('c',[]))==7 and s['c'][6]>0 for s in fs),'no v2 command time'
