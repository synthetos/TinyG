"""Master rejects metadata as a setting; exercise bootstrap through restore."""
import json
import runpy
from collections import deque
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from tinyg_hil.driver import TinyG
from tinyg_hil.protocol import parse_line
from tinyg_hil import snapshot

CONF = runpy.run_path(str(Path(__file__).parents[1] / 'conftest.py'))


class MasterWire:
    port = 'fake-master'
    is_open = True
    def __init__(self):
        self.events = deque()
        self.writes = []
        self.values = dict(fb=440.21, fv=.97, hp=1, hv=8, id='master', stat=4, qr=32,
                           js=0, jv=1, ee=1, ec=1, tv=0, sv=0, qv=1, si=250, sr={'stat':4})
        for m in range(1,5):
            self.values.update({f'{m}sa':1.8, f'{m}tr':1.25, f'{m}mi':8})
    def open(self): self.is_open = True
    def close(self): self.is_open = False
    def drain(self, *a, **kw):
        rows=list(self.events);self.events.clear();return rows
    def next_event(self, timeout): return self.events.popleft() if self.events else None
    def write(self, data):
        self.writes.append(data)
        line=data.decode().strip()
        if line.startswith('$'):
            token=line[1:]
            rows=[f'[{token}] value {self.values.get(token,0)}', 'tinyg [mm] ok>']
        elif line.startswith('{'):
            obj=json.loads(line);key=next(iter(obj));value=obj[key]
            if value is not None:self.values[key]=value
            body={key:self.values.get(key,0)}
            if 'tid' in obj:
                body['tid']=obj['tid']
                # Exact failure shape observed on master after a metadata SET.
                rows=[json.dumps({'r':body,'f':[1,100,18,2713]},separators=(',',':'))]
            else:
                rows=[json.dumps({'r':body,'f':[1,0,len(data),0]})]
        else: rows=['{"r":{},"f":[1,0,10,0]}']
        self.events.extend(parse_line(row,0) for row in rows)


@pytest.fixture
def master(monkeypatch):
    monkeypatch.setattr('tinyg_hil.driver.time.sleep',lambda _:None)
    tg=TinyG('fake-master');tg.transport=MasterWire();tg.connect()
    return tg


def test_exact_old_failure_remains_a_failure(master):
    response=master.send_line('{"1mi":2,"tid":1}',correlate=False)
    assert response.raw == '{"r":{"1mi":2,"tid":1},"f":[1,100,18,2713]}'
    assert response.status == 100


def test_real_session_baseline_keep_snapshot_and_restore(master,monkeypatch,tmp_path):
    # Run the production fixture, substituting only the physical port lock and wire.
    fn=CONF['tg'].__wrapped__
    class Lock:
        def __init__(self,*a):pass
        def acquire(self,*a,**kw):return self
        def release(self):pass
    monkeypatch.setitem(fn.__globals__,'BoardLock',Lock)
    monkeypatch.setitem(fn.__globals__,'TinyG',lambda *a,**kw:master)
    options={'--port':'fake-master','--baud':115200,'--allow-motion':True,
             '--snapshot-dir':tmp_path,'--lock-wait':0}
    request=NS(config=NS(getoption=lambda key:options.get(key),_tinyg_recorder=None,
                        invocation_params=NS(args=[])),addfinalizer=lambda f:None)
    original={k:master.transport.values[k] for k in CONF['BASELINE']}
    session=fn(request);assert next(session) is master
    for key,value in CONF['BASELINE'].items():assert master.get(key)==value
    keep=CONF['keep'].__wrapped__(master);save=next(keep);save('1mi','si')
    master.set('1mi',2,allow='offline');master.set('si',100,allow='offline')
    with pytest.raises(StopIteration):next(keep)
    assert master.get('1mi')==8 and master.get('si')==250
    saved=snapshot.take(master)
    master.set('1mi',2,allow='offline')
    assert not snapshot.restore(master,saved)['failed']
    assert master.get('1mi')==8
    with pytest.raises(StopIteration):next(session)
    assert all(master.transport.values[k]==v for k,v in original.items())
    assert all(b'"tid"' not in line for line in master.transport.writes)
