from types import SimpleNamespace as NS
import json
import pytest
from tinyg_hil import perf_snapshot as fixed


class Board:
    identity={'fb':449.01,'id':'bench'}
    def __init__(self):self.values={'4ma':0,'3ma':1,'st':0,'bjd':.05,'cjd':.05,'sr':{'stat':3}}
    def get(self,k):return self.values.get(k)
    def values_equal(self,k,a,b):return a==b


def snap():return dict(identity={'id':'bench'},values={'4ma':0,'3ma':1,'st':0,'bjd':.05,'cjd':.05},sr=['stat'])


def test_readback_rejects_default_motor_map_and_junction_values():
    b=Board();assert fixed.verify(b,snap())['ok']
    for token,value in [('4ma',3),('3ma',2),('st',1),('bjd',1),('cjd',1)]:
        b=Board();b.values[token]=value
        with pytest.raises(ValueError,match=token):fixed.verify(b,snap())


def test_snapshot_identity_and_explicit_comm_overrides():
    b=Board();s=snap();s['identity']['id']='other'
    with pytest.raises(ValueError):fixed.verify(b,s)
    s=snap();s['values']['qv']=1;b.values['qv']=0
    assert fixed.verify(b,s,overrides={'qv':0})['ok']
    with pytest.raises(ValueError):fixed.verify(b,s)


def test_legacy_restore_omits_only_declared_new_controls():
    s=snap();s['values'].update(spph=1,spde=1,coph=0,rtfi=1)
    filtered=fixed.for_board(s,449.01)
    assert filtered['values']==snap()['values']
    assert fixed.for_board(s,450.05)['values']==dict(snap()['values'],rtfi=1)


def test_flash_restore_always_uses_fixed_source_and_checks_failures(monkeypatch):
    from tinyg_hil import snapshot
    b=Board();s=snap();calls=[]
    def restore(tg,source,**kwargs):
        calls.append(source);tg.values.update(source['values'])
        return {'changed':list(source['values']),'failed':[],'dropped':[]}
    monkeypatch.setattr(snapshot,'restore',restore)
    for _ in range(2):
        b.values['4ma']=3
        assert fixed.restore_verified(b,s)['verification']['ok']
    assert all(c['values']['4ma']==0 for c in calls)
    monkeypatch.setattr(snapshot,'restore',lambda *a,**kw:{'failed':[('bjd',.05,108)]})
    with pytest.raises(ValueError):fixed.restore_verified(b,s)


def test_perf_flash_requires_fixed_source_before_opening_board(monkeypatch):
    from tinyg_hil import __main__ as cli
    monkeypatch.setenv('FIRMWARE_TAG','A');monkeypatch.delenv('PERF_SNAPSHOT',raising=False)
    monkeypatch.setattr(cli,'BoardLock',lambda *a:pytest.fail('opened board without fixed snapshot'))
    with pytest.raises(SystemExit):cli.main(['flash','--port','fake','--hex','fake.hex'])


def test_cli_each_flash_uses_same_fixed_file_not_new_snapshot(monkeypatch):
    from tinyg_hil import __main__ as cli
    from tinyg_hil import snapshot
    b=Board();b.dialect=NS(name='d449');b.connect=lambda:None;b.close=lambda:None
    monkeypatch.setattr(cli,'TinyG',lambda *a,**kw:b)
    lock=NS(release=lambda:None);lock.acquire=lambda *a,**kw:lock
    monkeypatch.setattr(cli,'BoardLock',lambda *a:lock)
    monkeypatch.setattr(fixed,'load',lambda path:(snap(),{'path':path,'sha256':'f'*64}))
    monkeypatch.setattr(snapshot,'take',lambda tg:{'values':{'4ma':3}})
    monkeypatch.setattr(snapshot,'save',lambda *a:'backup-only.json')
    def flash(*a,**kw):b.values['4ma']=3;return b.identity
    monkeypatch.setattr(cli.flasher,'flash',flash)
    sources=[]
    def restore(tg,source,**kwargs):
        sources.append(source);tg.values.update(source['values']);return {'failed':[]}
    monkeypatch.setattr(snapshot,'restore',restore)
    for _ in range(2):cli.main(['flash','--port','fake','--hex','fake.hex','--perf-snapshot','fixed.json'])
    assert len(sources)==2 and all(s['values']['4ma']==0 for s in sources)


def test_user_words_are_preserved_as_evidence_but_not_restored(monkeypatch):
    from tinyg_hil import snapshot
    b=Board();s=snap();s['values'].update(udb2='0x1',udb3='0x1',udc0='0x2')
    b.send_gcode=lambda *a,**kw:None;b.wait_idle=lambda:None
    b.dialect=NS(name='d449',tokens={k:{'persist':True,'type':'float' if k.endswith('jd') else 'integer'} for k in s['values']})
    b.set=lambda *a,**kw:pytest.fail('no setting should be written in this matching board')
    result=snapshot.restore(b,s)
    assert {r['token'] for r in result['not_restorable']}=={'udb2','udb3','udc0'}
    assert not result['failed']
    check=fixed.verify(b,s)
    assert check['ok'] and {r['token'] for r in check['not_restorable']}=={'udb2','udb3','udc0'}
    assert 'udb2' not in fixed.for_board(s,449.01)['values']
    assert s['values']['udb2']=='0x1' # original snapshot not edited


def test_legacy_print_precision_is_narrow_and_recorded():
    b=Board();b.values.update(bjd=0,cjd=0)
    result=fixed.verify(b,snap())
    assert result['ok']
    assert {r['token'] for r in result['reduced_precision']}=={'bjd','cjd'}
    assert all(r['precision']==0 and r['expected']==.05 and r['printed_expected']==0 for r in result['reduced_precision'])
    b.identity={'fb':450.04,'id':'bench'}
    with pytest.raises(ValueError,match='bjd'):fixed.verify(b,snap())
    b.identity={'fb':449.01,'id':'bench'};b.values['4ma']=3
    with pytest.raises(ValueError,match='4ma'):fixed.verify(b,snap())
    b=Board();b.values['bjd']=1
    with pytest.raises(ValueError,match='bjd'):fixed.verify(b,snap())


def test_legacy_restore_writes_real_value_then_checks_printed_value():
    b=Board();b.values.update(bjd=0,cjd=0);writes=[]
    b.send_gcode=lambda *a,**kw:None;b.wait_idle=lambda:None
    b.dialect=NS(name='d449',tokens={k:{'persist':True,'type':'float' if k.endswith('jd') else 'integer'} for k in snap()['values']})
    def set_value(token,value,**kw):
        writes.append((token,value));b.values[token]=round(value) if token in ('bjd','cjd') else value
    b.set=set_value
    result=fixed.restore_verified(b,snap())
    assert result['verification']['ok']
    assert writes==[('bjd',.05),('cjd',.05)]
