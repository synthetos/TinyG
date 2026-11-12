"""No board: signed capture metrics, protocol accounting and fixture directives."""
import json
from collections import deque
import pytest
from tinyg_hil import perf
from tinyg_hil.protocol import parse_line


def test_direction_and_pulses():
    r=perf.steps({'M1 step':[(1,1),(1.000002,0),(2,1),(2.000003,0)],'M1 dir':[(1.5,1)]},
                 {'M1 step':0,'M1 dir':0},{'M1':0})
    assert r['motors']['M1']['net']==0 and r['motors']['M1']['total']==2
    assert r['motion_s']==1
    assert r['motors']['M1']['min_pulse_s']==pytest.approx(.000002)
    with pytest.raises(ValueError): perf.steps({'M1 step':[]},{'M1 step':0},{'M1':0})
    with pytest.raises(ValueError): perf.steps({'M1 step':[(1,1)],'M1 dir':[]},{'M1 step':0,'M1 dir':0},{'M1':0})


def test_old_dialect_options():
    assert perf.options({'FIRMWARE_TAG':'A','SLOT':'1'})==('A','1','off')
    with pytest.raises(ValueError): perf.options({'FIRMWARE_TAG':'A','SLOT':'1','RT':'on'})
    with pytest.raises(ValueError): perf.options({})


class Fake:
    idle_stats={1,3,4}
    def __init__(self, fault=None):
        self.t=0.;self.events=deque();self.writes=[];self.transport=self;self.dialect=self
        self.held=False;self.fault=fault
    def clock(self): return self.t
    def get(self,k): assert k=='rx';return 8
    def stat(self,k): return {'HOLD':6,'ALARM':2,'SHUTDOWN':12,'PANIC':13}[k]
    def stats(self,*ks): return {self.stat(k) for k in ks}
    def write(self,data,**kw):
        self.writes.append((self.t,data));self.t+=.001
        if data in (b'!',b'~'): self.held=data==b'!';return
        if data.startswith(b'{'):
            tid=json.loads(data)['tid']
            if self.fault=='no_probe': return
            r={'r':{'stat':6 if self.held else 3},'tid':tid,'f':[3,0,8]}
        else: r={'r':{},'f':[3,110 if self.fault=='refusal' else 0,8]}
        self.events.append(parse_line(json.dumps(r),self.t))
    def next_event(self,timeout):
        self.t+=timeout
        return self.events.popleft() if self.events else None


def test_probe_replies_are_not_gcode_acks():
    f=Fake();r=perf.stream(f,['G1 X1','G1 X2'],3,clock=f.clock)
    assert r['completion']=='idle' and not r['errors'] and r['acked']==2
    assert len(r['probes'])>=3
    assert all(p['rtt_s']>=0 for p in r['probes'])
    assert len(r['ack_rows'])==2


def test_directives_wait_and_resume():
    f=Fake();r=perf.stream(f,['G1 X60',';HOST-PAUSE 0.5',';HOLD-RESUME 0.1 0.5','G1 X0'],5,clock=f.clock)
    assert not r['errors'] and r['completion']=='idle' and r['acked']==2
    bang=next(t for t,d in f.writes if d==b'!');resume=next(t for t,d in f.writes if d==b'~')
    assert bang>=.6 and resume-bang>=.5
    assert not any(d.startswith(b';') for _,d in f.writes)


@pytest.mark.parametrize('fault',['refusal','no_probe'])
def test_failed_stream_never_passes(fault):
    f=Fake(fault);r=perf.stream(f,['G1 X1'],7,clock=f.clock)
    assert r['errors'] and r['completion']!='idle'


def test_snapshot_never_queries_new_controls_on_449():
    class Board:
        dialect=None
        def __init__(self):self.dialect=self;self.queries=[]
        def persisted_tokens(self):return ['fb','xfr','spph','spde','coph','rtfi','_hld','se00']
        def send_gcode(self,*a,**k):pass
        def wait_idle(self):pass
        def get(self,k):self.queries.append(k);return 800 if k=='xfr' else {}
    tg=Board()
    assert perf.settings_snapshot(tg)['values']=={'xfr':800}
    assert tg.queries==['xfr','sr']


def test_firmware_preflight_rejects_wrong_tag_and_hash():
    flash={'commit':'abc','hex_sha256':'a'*64}
    with pytest.raises(ValueError,match='449.01'):
        perf.validate_firmware({'fb':450.04},flash,'A',{})
    with pytest.raises(ValueError):
        perf.validate_firmware({'fb':450.03},flash,'B',{})
    with pytest.raises(ValueError,match='hex'):
        perf.validate_firmware({'fb':450.04},flash,'B',{'PERF_EXPECT_HEX':'b'*64})
    with pytest.raises(ValueError):
        perf.validate_firmware({'fb':450.04},{},'B',{'PERF_EXPECT_HEX':'a'*64})
    result=perf.validate_firmware({'fb':449.01},flash,'A',{})
    assert result['flash']['known'] and result['flash']['commit']=='abc'
    assert perf.validate_firmware({'fb':450.05},flash,'B',{'PERF_EXPECT_FB':'450.05'})['expected_fb']==450.05
    assert not perf.validate_firmware({'fb':450.04},{},'B',{})['flash']['known']


@pytest.mark.parametrize('fault',['firmware','settings'])
def test_collector_preflight_stops_before_settings_motion_or_capture(monkeypatch,tmp_path,fault):
    import importlib.util
    from pathlib import Path
    from types import SimpleNamespace as NS
    spec=importlib.util.spec_from_file_location('perf_diagnostic',Path(__file__).parent/'diagnostics/test_perf_compare.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setenv('FIRMWARE_TAG','A');monkeypatch.setenv('SLOT','offline')
    monkeypatch.setattr(module.benches,'bench',lambda port:{'name':'fake','mso':{'digital':{
        str(i):f'M{m} {k}' for i,(m,k) in enumerate((m,k) for m in range(1,5) for k in ('step','dir'))}}})
    monkeypatch.setattr(module,'read_provenance',lambda port:{'hex_sha256':'b'*64,'commit':'candidate'})
    class Board:
        identity={'fb':450.04}
        transport=NS(port='fake')
        def get(self,k):return {'fb':450.04 if fault=='firmware' else 449.01,'4ma':3,'sr':{'stat':3}}[k]
        def reset(self):pytest.fail('preflight must not reset the wrong firmware')
    def no_settings(*a):pytest.fail('settings changed before firmware check')
    monkeypatch.setattr(module.perf_snapshot,'load',lambda path:({'values':{'4ma':0},'sr':['stat']},{'sha256':'f'*64}))
    request=NS(config=NS(_tinyg_recorder=NS(dir=tmp_path)))
    with pytest.raises(ValueError,match='requires fb 449.01' if fault=='firmware' else 'fixed snapshot mismatch'):
        module.test_perf_compare(Board(),no_settings,lambda *a:None,request,'01_long_straight_x.nc')
    record=json.loads(next(tmp_path.glob('perf-*.json')).read_text())
    assert not record['valid'] and record['identity']['fb']==(450.04 if fault=='firmware' else 449.01)
    assert record['flash']['known'] and record['flash']['hex_sha256']=='b'*64


def test_no_probes_uses_automatic_idle_report_and_records_credit():
    class Automatic(Fake):
        def write(self,data,**kw):
            assert not data.startswith(b'{')
            super().write(data,**kw)
            self.events.append(parse_line('{"sr":{"stat":3}}',self.t))
    f=Automatic();r=perf.stream(f,['G1 X1','G1 X2'],3,clock=f.clock,probe_interval=None)
    assert r['completion']=='idle' and r['acked']==2
    assert r['probe_mode']=='off' and r['probes']==[]
    assert r['rx_credit_min']==8 and r['rx_credit_last']==8


def test_credit_min_includes_priority_replies():
    class Leaking(Fake):
        def write(self,data,**kw):
            super().write(data,**kw)
            if data.startswith(b'{'):
                ev=self.events.pop()
                raw=json.loads(ev.raw);raw['f'][2]=2
                self.events.append(parse_line(json.dumps(raw),self.t))
    f=Leaking();r=perf.stream(f,['G1 X1'],3,clock=f.clock)
    assert r['rx_credit_min']==2 and r['probe_mode']=='250ms'


def test_probe_options_default_off_and_b_only():
    assert perf.probe_options({'FIRMWARE_TAG':'A'}) is None
    assert perf.probe_options({'FIRMWARE_TAG':'B'}) is None
    assert perf.probe_options({'FIRMWARE_TAG':'B','PERF_PROBES':'on'}) == .25
    with pytest.raises(ValueError):perf.probe_options({'FIRMWARE_TAG':'A','PERF_PROBES':'on'})
    with pytest.raises(ValueError):perf.probe_options({'FIRMWARE_TAG':'B','PERF_PROBES':'typo'})


def test_449_hold_uses_post_bang_zero_velocity_and_settle():
    class Old(Fake):
        def __init__(self):
            super().__init__();self.identity={'fb':449.01}
        def write(self,data,**kw):
            super().write(data,**kw)
            if data==b'!':
                self.events.append(parse_line('{"sr":{"stat":4,"vel":0}}',self.t))
            elif data.startswith(b'G'):
                self.events.append(parse_line('{"sr":{"stat":4,"vel":0}}',self.t))
    f=Old();r=perf.stream(f,['G1 X60',';HOLD-RESUME 0.1 0.2','G1 X0'],5,clock=f.clock,probe_interval=None)
    assert r['completion']=='idle'
    bang=next(t for t,d in f.writes if d==b'!');resume=next(t for t,d in f.writes if d==b'~')
    assert resume-bang>=.7
    assert r['hold_detection']=='449 post-! vel=0 plus 0.5 s settle'


def test_449_stale_idle_status_does_not_finish_while_moving():
    class Moving(Fake):
        identity={'fb':449.01}
        def write(self,data,**kw):
            super().write(data,**kw)
            self.events.append(parse_line('{"sr":{"stat":4,"vel":800}}',self.t))
    f=Moving();r=perf.stream(f,['G1 X60'],1,clock=f.clock,probe_interval=None)
    assert r['completion']=='timeout'


def test_capture_budget_preserves_two_mhz_limit():
    assert perf.capture_budget('mso',600,None)==(500,498)
    assert perf.capture_budget('mso',37,None)==(37,35)
    with pytest.raises(ValueError):perf.capture_budget('mso',600,550)


@pytest.mark.parametrize('fb',[449.01,450.04])
def test_zero_before_hold_cannot_trigger_legacy_resume_or_replace_b_hold(fb):
    class Board(Fake):
        identity={'fb':fb}
        def write(self,data,**kw):
            super().write(data,**kw)
            if data.startswith(b'G') or (data==b'!' and fb>=450):
                self.events.append(parse_line('{"sr":{"stat":4,"vel":0}}',self.t))
    f=Board();r=perf.stream(f,['G1 X60',';HOLD-RESUME 0.1 0.1','G1 X0'],2,clock=f.clock,probe_interval=None)
    assert r['completion']=='timeout'
    assert not any(data==b'~' for _,data in f.writes)


def test_step_legs_count_only_direction_changes_with_pulses():
    r=perf.steps({'M1 step':[(1,1),(1.1,0),(2,1),(2.1,0),(3,1),(3.1,0)],
                  'M1 dir':[(1.5,1),(1.6,0),(2.5,1)]},
                 {'M1 step':0,'M1 dir':0},{'M1':0})
    assert r['motors']['M1']['reversal_legs']==2


def test_probe_credit_recovery_restarts_a_drained_data_window():
    class Fragmented(Fake):
        def write(self,data,**kw):
            super().write(data,**kw)
            if not data.startswith(b'{'):
                ev=self.events.pop();raw=json.loads(ev.raw);raw['f'][2]=0
                self.events.append(parse_line(json.dumps(raw),self.t))
    f=Fragmented();lines=[f'G1 X{i}' for i in range(22)]
    r=perf.stream(f,lines,4,clock=f.clock)
    assert r['completion']=='idle' and not r['errors']
    assert r['sent']==r['acked']==22 and r['unsent']==r['pending']==0
    assert [row['index'] for row in r['ack_rows']]==list(range(22))
    assert [row['line'] for row in r['sent_rows']]==lines
    assert [data.decode().strip() for _,data in f.writes if data.startswith(b'G')]==lines
    assert any(row['free']==0 and not row['probe'] for row in r['rx_credit_rows'])
    assert any(row['free']==8 and row['probe'] for row in r['rx_credit_rows'])


def test_zero_probe_credit_does_not_authorize_more_data():
    class Full(Fake):
        def write(self,data,**kw):
            super().write(data,**kw)
            ev=self.events.pop();raw=json.loads(ev.raw);raw['f'][2]=0
            self.events.append(parse_line(json.dumps(raw),self.t))
    f=Full();r=perf.stream(f,[f'G1 X{i}' for i in range(22)],1,clock=f.clock)
    assert r['completion']=='timeout'
    assert r['sent']==r['acked']==6 and r['pending']==0 and r['unsent']==16
