"""Flat override settings use the same motion and lifecycle checks as rt aliases."""
import json
from types import SimpleNamespace

import pytest
import test_live_feed_override_body as body
import test_live_feed_override_lifecycle as lifecycle
from test_feed_advisor_step3 import rt

pytestmark = [pytest.mark.dialect('d449'), pytest.mark.resets_board]


def flat(tg, fields):
    # Event and byte controls remain rt-only. Normalize only the response container
    # for reuse of the same hardware assertions; all setting traffic is flat JSON.
    if any(k in ('ov', 'ovb') for k in fields):
        return rt(tg, fields)
    names={k:('mfoe' if k=='mfe' and v is not None else k) for k,v in fields.items()}
    reply=tg.send_line(json.dumps({names[k]:v for k,v in fields.items()}))
    return SimpleNamespace(status=reply.status,body={'rt':{k:reply.body.get(n) for k,n in names.items()}})


@pytest.mark.motion
@pytest.mark.persists
@pytest.mark.timeout(90)
@pytest.mark.parametrize('factor',[.5,1.5])
def test_flat_body(tg,keep,record,monkeypatch,factor):
    tg.reset()
    monkeypatch.setattr(body,'rt',flat)
    body.test_long_body_changes_speed_and_keeps_endpoint(tg,keep,record,factor)


@pytest.mark.motion
@pytest.mark.timeout(45)
def test_flat_program_permission(tg,record,monkeypatch):
    monkeypatch.setattr(lifecycle,'rt',flat)
    lifecycle.test_program_permission_cannot_be_bypassed_by_live_request(tg,record)


@pytest.mark.motion
@pytest.mark.timeout(45)
@pytest.mark.parametrize('ending',['M2','M30','kill','reset'])
def test_flat_end(tg,record,monkeypatch,ending):
    monkeypatch.setattr(lifecycle,'rt',flat)
    lifecycle.test_job_end_restores_unity_permission_and_disables_bytes(tg,ending,record)
    assert tg.get('m48e')==1
    assert tg.get('mfoe')==1


@pytest.mark.parametrize('word,expected', [('0.05',.05),('0.050',.05),('5e-2',.05),('0.051',.051),('2.0',2.0)])
def test_flat_factor(tg,word,expected):
    try:
        tg.reset()
        reply=tg.send_line('{"mfo":'+word+'}')
        assert reply.status==0
        assert reply.body['mfo']==pytest.approx(expected)
        assert tg.get('mfo')==pytest.approx(expected)
        assert rt(tg,{'mfo':None}).body['rt']['mfo']==pytest.approx(expected)
    finally:tg.reset()


@pytest.mark.parametrize('word',['0.049','0.04999999','2.001'])
def test_flat_range_echo(tg,word):
    try:
        tg.reset()
        reply=tg.send_line('{"mfo":'+word+'}')
        assert reply.status==110
        assert reply.body['mfo']==pytest.approx(round(float(word),3))
        assert tg.get('mfo')==1
    finally:tg.reset()


@pytest.mark.motion
@pytest.mark.timeout(45)
def test_master_and_feed_switches_do_not_bypass_program_permission(tg):
    try:
        tg.reset()
        for command in ('M49','M50 P0'):
            tg.send_gcode(command,check=True)
            lifecycle.wait_value(tg,'mfe',0)
            for master,feed in ((0,1),(1,0),(0,0),(1,1)):
                assert tg.send_line(json.dumps({'m48e':master,'mfoe':feed})).status==0
                assert tg.get('m48e')==master and tg.get('mfoe')==feed
                assert tg.get('mfe')==0
        tg.send_gcode('M48',check=True)
        lifecycle.wait_value(tg,'mfe',1)
        for master,feed in ((0,1),(1,0),(0,0),(1,1)):
            assert tg.send_line(json.dumps({'m48e':master,'mfoe':feed})).status==0
            assert tg.get('mfe')==(master and feed)
    finally:tg.reset()


@pytest.mark.parametrize('ending',['M2','M30','kill','reset'])
def test_flat_master_and_feed_reset(tg,ending):
    try:
        tg.reset()
        assert tg.send_line('{"m48e":0,"mfoe":0,"mfo":0.5}').status==0
        if ending=='reset':tg.reset()
        elif ending=='kill':tg.send_raw(b'\x04')
        else:tg.send_gcode(ending,check=True)
        lifecycle.wait_value(tg,'mfo',1)
        assert tg.get('m48e')==1 and tg.get('mfoe')==1
        assert tg.get('mfe')==1
    finally:tg.reset()


@pytest.mark.parametrize('token,value',[('mfo',.5),('mfoe',0),('m48e',0)])
def test_text_setting_and_display(tg,token,value):
    try:
        tg.reset()
        lines=tg.send_text(f'${token}={value}')
        assert not any('err:' in line for line in lines),lines
        assert tg.get(token)==pytest.approx(value)
        lines=tg.send_text(f'${token}')
        assert any(f'[{token}]' in line for line in lines),lines
    finally:tg.reset()
