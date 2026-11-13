"""Run the upgrade orchestration and real flash helper against a fake wire."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import subprocess
import json
from tinyg_hil.state import state_dir

import pytest
from tinyg_hil import flash, snapshot, bootloader
from tinyg_hil.protocol import loads

spec=importlib.util.spec_from_file_location('layout3_case',Path(__file__).parent/'bugs/test_layout3_upgrade.py')
case=importlib.util.module_from_spec(spec)
spec.loader.exec_module(case)


class Wire:
    port='fake-upgrade-port'
    def __init__(self):
        self.is_open=True
        self.sent=[]
        self.opens=0
    def open(self):
        bootloader.require_application(self.port)
        self.is_open=True
        self.opens+=1
    def close(self):self.is_open=False
    def write(self,data):
        bootloader.require_application(self.port)
        assert self.is_open, 'attempted write on a closed transport'
        self.sent.append(data)


class Board:
    def __init__(self,old_fb):
        self.transport=Wire()
        self.values={'1sa':1.8,'prbp':0,'xvm':800}
        self.fields=['stat','posx']
        self.old_fb=old_fb
        self.old=False
        self.boot=False
        self.identity={'id':'fake','fb':450.04}
        self.resets=0
        self.dialect=NS(name='d449',status=lambda key: {'FUNCTION_IS_STUBBED':17,'UNRECOGNIZED_NAME':100}[key],
                        persisted_tokens=lambda:list(self.values))
        self.refresh()
    def refresh(self):
        self.dialect.tokens={key:{'persist':True,'type':'float'} for key in self.values}
    def connect(self,**kwargs):
        if not self.transport.is_open:self.transport.open()
        return self.identity
    def close(self):self.transport.close()
    def set(self,key,value,**kwargs):
        self.transport.write(f'{key}={value}'.encode())
        self.values[key]=value
        self.refresh()
        return NS(status=0)
    def get(self,key):
        self.transport.write(f'get:{key}'.encode())
        return dict.fromkeys(self.fields,0) if key=='sr' else self.values[key]
    def values_equal(self,token,a,b):return a==b
    def send_gcode(self,*args,**kwargs):self.transport.write(b'G21')
    def wait_idle(self):pass
    def reset(self):
        self.transport.write(b'\x18')
        self.resets+=1
    def send_json(self,obj,**kwargs):
        self.transport.write(str(obj).encode())
        if isinstance(obj,str):obj=loads(obj.replace(":t",":true"))
        if 'sr' in obj:self.fields=list(obj['sr']);return NS(status=0)
        key,value=next(iter(obj.items()))
        if key in case.REMOVED:
            if not self.old:return NS(status=100)
            if self.old_fb==450.03:return NS(status=17)
        if value is not None:self.set(key,value)
        return NS(status=0)


@pytest.fixture
def rig(monkeypatch,tmp_path):
    def make(old_fb=450.04,fail=(),timeout=False,reconnect_failure=False,corrupt=None):
        board=Board(old_fb)
        calls=[]
        saved=[]
        monkeypatch.setenv('TINYG_LAYOUT2_SOURCE',str(tmp_path/'old'))
        monkeypatch.setenv('TINYG_HIL_STATE',str(tmp_path/'state'))
        monkeypatch.setattr(flash,'build',lambda src:Path(src)/'tinyg.hex')
        monkeypatch.setattr(flash,'source_build',lambda src:old_fb if Path(src).name=='old' else 450.04)
        monkeypatch.setattr(flash.time,'sleep',lambda _:None)
        monkeypatch.setattr(flash,'flash_provenance',lambda path:{'hex':str(path)})
        monkeypatch.setattr(flash,'write_flash_record',lambda *a:None)
        class Raw:
            def write(self, data):
                assert data == b'\n{"boot":1}\n'
                assert not board.boot, 'application text sent to AVR109'
                board.boot=True
            def close(self): pass
        monkeypatch.setattr(bootloader, 'inspect_port', lambda *a: (Raw(), board.boot))
        def program(cmd,**kwargs):
            assert not board.transport.is_open, 'avrdude must own the closed port'
            calls.append(cmd)
            if len(calls) in fail:
                if timeout:raise subprocess.TimeoutExpired(cmd,kwargs['timeout'])
                return NS(returncode=1,stderr='flash verification mismatch at addr 0x0002')
            board.boot=False
            board.old=str(tmp_path/'old') in cmd[-1]
            board.identity['fb']=old_fb if board.old else 450.04
            if not board.old:
                board.values={k:v for k,v in board.values.items() if k not in case.REMOVED}
            if len(calls)==2 and corrupt:
                if corrupt=='sr':board.fields=['stat']
                else:board.values[corrupt]=0
            board.refresh()
            return NS(returncode=0,stderr='')
        monkeypatch.setattr(flash.subprocess,'run',program)
        real_restore=snapshot.restore
        def restore(tg,snap,**kwargs):
            saved.append((tg.old,dict(snap['values'])))
            return real_restore(tg,snap,**kwargs)
        monkeypatch.setattr(snapshot,'restore',restore)
        if reconnect_failure:
            board.connect=lambda **kw: (_ for _ in ()).throw(RuntimeError('connect failed'))
        return board,calls,saved
    return make


@pytest.mark.parametrize('old_fb',[450.03,450.04])
def test_old_flash_upgrade_verify_and_restore(rig,old_fb):
    board,calls,restores=rig(old_fb)
    records={}
    case.test_layout2_upgrade_preserves_settings_and_report_fields(board,lambda k,v:records.update({k:v}))
    assert len(calls)==2, 'no third flash after a verified upgrade'
    assert board.transport.is_open and board.transport.opens==2
    assert records['layout3_upgrade']['verified'] and board.resets==1
    before=records['layout3_upgrade']['before']
    assert before['values']['1sa']==.9 and before['values']['prbp']==1
    if old_fb==450.04:assert before['values']['spph']==1
    assert [old for old,_ in restores]==[True,False]
    assert board.values=={'1sa':1.8,'prbp':0,'xvm':800}
    assert board.fields==['stat','posx']


@pytest.mark.parametrize('failed_flash',[1,2])
@pytest.mark.parametrize('timeout',[False,True])
def test_failed_flash_uses_avr109_recovery_and_preserves_original_error(rig,failed_flash,timeout):
    board,calls,_=rig(fail=(failed_flash,),timeout=timeout)
    error=subprocess.TimeoutExpired if timeout else RuntimeError
    with pytest.raises(error) as caught:
        case.test_layout2_upgrade_preserves_settings_and_report_fields(board,lambda *a:None)
    if not timeout:assert 'verification mismatch' in str(caught.value)
    assert len(calls)==failed_flash+1 and board.transport.is_open
    assert not board.old and board.values['1sa']==1.8
    board.reset()  # session teardown has a usable transport


def test_recovery_failure_does_not_replace_primary_flash_error(rig):
    board,calls,_=rig(fail=(1,2))
    with pytest.raises(RuntimeError,match='verification mismatch') as caught:
        case.test_layout2_upgrade_preserves_settings_and_report_fields(board,lambda *a:None)
    assert any('recovery also failed' in note for note in caught.value.__notes__)
    assert not board.transport.is_open
    with pytest.raises(bootloader.BootloaderActive):board.reset()


def test_flash_can_start_with_closed_transport(rig):
    board,_,_=rig()
    board.close()
    flash.flash(board,case.SOURCE/'tinyg.hex',expect_fb=450.04)
    assert board.transport.is_open and board.transport.opens==1


def test_connect_failure_blocks_teardown_traffic(rig):
    board,_,_=rig(reconnect_failure=True)
    with pytest.raises(RuntimeError,match='connect failed'):
        flash.flash(board,case.SOURCE/'tinyg.hex')
    assert not board.transport.is_open
    with pytest.raises(bootloader.BootloaderActive):board.reset()


def test_timeout_interrupt_before_programming_keeps_quarantine(rig,monkeypatch):
    board,_,_=rig()
    class Interrupted(BaseException):pass
    def sleep(seconds):
        if seconds==.5:raise Interrupted('test deadline')
    monkeypatch.setattr(flash.time,'sleep',sleep)
    with pytest.raises(Interrupted,match='test deadline'):
        flash.flash(board,case.SOURCE/'tinyg.hex')
    assert not board.transport.is_open
    with pytest.raises(bootloader.BootloaderActive):board.reset()


@pytest.mark.parametrize('corrupt',['prbp','sr'])
def test_verify_detects_loss_before_restoring_original_settings(rig,corrupt):
    board,calls,restores=rig(corrupt=corrupt)
    with pytest.raises(AssertionError):
        case.test_layout2_upgrade_preserves_settings_and_report_fields(board,lambda *a:None)
    assert len(calls)==2 and len(restores)==2
    assert board.values['prbp']==0 and board.fields==['stat','posx']


def test_upgrade_has_budget_for_two_complete_flashes():
    timeout=next(mark for mark in case.pytestmark if mark.name=='timeout')
    assert timeout.args[0]>=2*180+2*30+120


@pytest.mark.parametrize('failure', ['none', 'verify', 'timeout'])
def test_each_programming_attempt_has_durable_full_output(rig, monkeypatch, failure):
    board, calls, _ = rig(fail=(1,) if failure != 'none' else (), timeout=failure == 'timeout')
    program = flash.subprocess.run
    def observed(cmd, **kwargs):
        logs = list((state_dir()/'flash-attempts').glob('*.json'))
        assert any(json.loads(p.read_text())['state'] == 'started' for p in logs)
        return program(cmd, **kwargs)
    monkeypatch.setattr(flash.subprocess, 'run', observed)
    if failure == 'none':
        case.test_layout2_upgrade_preserves_settings_and_report_fields(board, lambda *a: None)
    else:
        with pytest.raises((RuntimeError, subprocess.TimeoutExpired)):
            case.test_layout2_upgrade_preserves_settings_and_report_fields(board, lambda *a: None)
    entries = [json.loads(p.read_text()) for p in (state_dir()/'flash-attempts').glob('*.json')]
    assert len(entries) == len(calls) == 2
    assert all(e['command'] in calls and e['timeout'] == 180 for e in entries)
    assert all('stdout' in e and 'stderr' in e for e in entries)
    if failure == 'verify':
        assert any('verification mismatch' in e['stderr'] for e in entries)
    if failure == 'timeout':
        assert any(e['state'] == 'interrupted' and 'TimeoutExpired' in e['error'] for e in entries)
    assert not board.old and board.transport.is_open


def test_timeout_log_retains_partial_byte_output(rig, monkeypatch):
    board, _, _ = rig()
    def timeout(cmd, **kwargs):
        raise subprocess.TimeoutExpired(cmd, kwargs['timeout'], output=b'write started', stderr=b'partial verify')
    monkeypatch.setattr(flash.subprocess, 'run', timeout)
    with pytest.raises(subprocess.TimeoutExpired):
        flash.flash(board, case.SOURCE/'tinyg.hex')
    entry = json.loads(next((state_dir()/'flash-attempts').glob('*.json')).read_text())
    assert entry['stdout'] == 'write started' and entry['stderr'] == 'partial verify'
    assert not board.transport.is_open
    with pytest.raises(bootloader.BootloaderActive):board.reset()
