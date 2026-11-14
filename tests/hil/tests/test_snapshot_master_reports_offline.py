"""Restore cross-version report selections without unsupported master fields."""
import json
import re
import pytest
from tinyg_hil import snapshot
from tinyg_hil.driver import StatusError
from tinyg_hil.protocol import Footer, Response, parse_line
from tinyg_hil.dialect import load


class Board:
    def __init__(self, name='d440', unknown=()):
        self.dialect=load(name);self.fields=['stat'];self.unknown=set(unknown)
        self.writes=[];self.reads=[]
    def get(self, key):
        self.reads.append(key)
        if key in self.unknown:
            event=parse_line(json.dumps({'r':{},'f':[1,100,0,0]}),0)
            raise StatusError(Response(key,event,Footer.from_event(event),[],0),'unknown')
        return dict.fromkeys(self.fields,0) if key=='sr' else 0
    def send_gcode(self,*a,**kw):pass
    def wait_idle(self):pass
    def send_json(self,line,**kw):
        from tinyg_hil.protocol import loads
        fields=list(loads(re.sub(r':t(?=[,}])', ':true', line))['sr'])
        assert not self.unknown.intersection(fields)
        self.fields=fields;self.writes.append(line)


def test_drops_edge_fields_and_probes_master_variants():
    board=Board(unknown=['hold'])
    saved=['stat','hold','posx','spe','spc','sps','cof','com']
    result=snapshot.restore(board,{'values':{},'sr':saved})
    assert board.fields==['stat','posx'] and len(board.writes)==1
    assert result['dropped']==['sr.'+key for key in ['hold','spe','spc','sps','cof','com']]
    assert result['changed']==['sr'] and not result['failed']
    assert 'spe' not in board.reads  # absent tokens need no request


def test_supported_hold_is_not_dropped_by_name():
    board=Board()
    result=snapshot.restore(board,{'values':{},'sr':['stat','hold','posx']})
    assert not result['dropped'] and board.fields==['stat','hold','posx']


def test_all_unsupported_retains_current_report_and_records_it():
    board=Board()
    result=snapshot.restore(board,{'values':{},'sr':['spe','com']})
    assert board.fields==['stat'] and not board.writes
    assert result['dropped']==['sr.spe','sr.com'] and result['sr_retained']


def test_edge_does_not_drop_new_fields_missing_from_old_catalog():
    board=Board('d449')
    result=snapshot.restore(board,{'values':{},'sr':['stat','spe']})
    assert board.fields==['stat','spe'] and not result['dropped']


def test_action_get_is_not_executed_during_filter():
    board=Board()
    result={'dropped':[]}
    snapshot._master_report_fields(board,['defa'],result)
    assert 'defa' not in board.reads


def test_other_read_errors_are_not_treated_as_missing_fields(monkeypatch):
    board=Board()
    def get(key):
        event=parse_line('{"r":{},"f":[1,204,0,0]}',0)
        raise StatusError(Response(key,event,Footer.from_event(event),[],0),'alarm')
    monkeypatch.setattr(board,'get',get)
    with pytest.raises(StatusError):
        snapshot.restore(board,{'values':{},'sr':['stat','hold']})
    assert not board.writes
