"""Request correlation against delayed, stale and asynchronous board output."""
import json
from collections import deque
import pytest
from tinyg_hil.driver import TinyG, NoResponse, StatusError
from tinyg_hil.protocol import parse_line

class FakeTransport:
    def __init__(self, stale=(), answer=None):
        self.events = deque(parse_line(s, 0) for s in stale)
        self.writes, self.answer = [], answer
    def write(self, data):
        self.writes.append(data)
        if self.answer:
            self.events.extend(parse_line(s, 1) for s in self.answer(data))
    def next_event(self, timeout):
        return self.events.popleft() if self.events else None

def reply(tid=None, status=0):
    obj = {"r": {}, "f": [3, status, 10]}
    if tid is not None: obj["tid"] = tid
    return json.dumps(obj)

def driver(stale=(), answer=None):
    tg = TinyG("fake")
    tg.transport = FakeTransport(stale, answer)
    return tg

def test_json_waits_for_own_reply_and_records_stale_output():
    tg = driver([reply(), reply(99)], lambda data: [reply(100), '{"sr":{"stat":3}}', reply(json.loads(data)["tid"])])
    resp = tg.send_line('{"ja":100001}')
    assert json.loads(tg.transport.writes[0])["tid"] == 1
    assert resp.event.data["tid"] == 1
    assert len(resp.events) == 4
    assert resp.sent == '{"ja":100001}'

def test_json_ids_increment_and_get_does_not_pause(monkeypatch):
    sleeps = []
    monkeypatch.setattr("tinyg_hil.driver.time.sleep", sleeps.append)
    tg = driver(answer=lambda data: [reply(json.loads(data)["tid"])] if "tid" in json.loads(data) else [keyed("stat", 3)])
    tg.send_json({"stat": None})
    tg.send_json({"ja": 100001}, allow="test")
    assert [json.loads(data).get("tid") for data in tg.transport.writes] == [None, 1]
    assert sleeps == [0.05]

def test_missing_matching_tid_times_out_even_with_success_replies():
    tg = driver([reply(), reply(9)], lambda data: [reply(8)])
    with pytest.raises(NoResponse): tg.send_line('{"ja":100001}', timeout=0.01)

def test_gcode_drains_queued_backlog_and_skips_delayed_tagged_reply():
    tg = driver([reply(9), reply()], lambda data: [reply(10), reply(status=247)])
    resp = tg.send_line('G1 X1 F500')
    assert tg.transport.writes == [b'G1 X1 F500\n']
    assert resp.status == 247
    assert len(resp.events) == 3

def test_matched_error_still_raises_check():
    tg = driver([reply(status=0)], lambda data: [reply(json.loads(data)["tid"], 13)])
    with pytest.raises(StatusError): tg.send_line('{"ja":100001}', check=True)

def test_relaxed_or_malformed_json_keeps_payload_bytes():
    tg = driver(answer=lambda data: [reply(status=48)])
    tg.send_line('  {xvm:1')
    assert tg.transport.writes == [b'  {xvm:1\n']

def test_explicit_raw_protocol_escape_hatch():
    tg = driver(answer=lambda data: [reply(42)])
    tg.send_json('{"fb":null,"tid":42}', correlate=False)
    assert tg.transport.writes == [b'{"fb":null,"tid":42}\n']

# Bootstrap/recovery output is deliberately untagged. Firmware parses a numeric
# tid as a SET, so it must follow an alarm-clearing GET rather than precede it.
def keyed(key, value, tid=None, status=0):
    data = {"r": {key: value}, "f": [3, status, 10]}
    if tid is not None: data["tid"] = tid
    return json.dumps(data)


def banners():
    return [json.dumps({"sr": {"msg": msg, "fb": 450.01}})
            for msg in ["Loading configs from EEPROM", "SYSTEM READY"]]


def test_bootstrap_fb_is_untagged_and_skips_unrelated_output():
    def answer(data):
        assert data == b'{"fb":null}\n'
        return banners() + ['{"qr":32}', keyed("stat", 5), keyed("fb", 450.01)]
    tg = driver(answer=answer)
    assert tg._responds()
    assert tg._next_tid == 1


@pytest.mark.parametrize("state", [2, 5])
@pytest.mark.parametrize("text_mode", [False, True])
def test_connect_uses_plain_keyed_identity_reads_in_alarm_or_motion(state, text_mode):
    values = {"fb": 450.01, "fv": 0.97, "hp": 1, "hv": 8, "id": "board"}
    def answer(data):
        obj = json.loads(data)
        assert "tid" not in obj
        key = next(iter(obj))
        return (["tinyg [mm] ok>"] if text_mode else []) + [json.dumps({"sr": {"stat": state}}), keyed(key, values[key])]
    tg = driver(answer=answer)
    tg.transport.is_open = False
    tg.transport.open = lambda: setattr(tg.transport, "is_open", True)
    tg.transport.drain = lambda *args, **kwargs: []
    assert tg.connect() == values
    assert tg.dialect.name == "d449"
    assert all(b'\x18' not in data for data in tg.transport.writes)


def test_reset_waits_for_system_ready_not_loading_banner():
    tg = driver(answer=lambda data: banners() if data == b'\x18' else [])
    tg.transport.drain = lambda *args, **kwargs: []
    assert tg.reset().body["msg"] == "SYSTEM READY"
    assert tg.transport.writes == [b'\x18', b'{"fb":null}\n']      # the reset ends by switching the board to JSON


def test_wake_retries_plain_fb_after_xon_and_ready_reset():
    reset = False
    def answer(data):
        nonlocal reset
        if data == b'\x18':
            reset = True
            return banners()
        if data == b'\x11\n': return [reply()]
        assert data == b'{"fb":null}\n'
        return [keyed("fb", 450.01)] if reset else []
    tg = driver(answer=answer)
    tg.transport.drain = lambda *args, **kwargs: []
    tg._wake()
    assert tg.transport.writes == [b'\x11\n', b'{"fb":null}\n', b'\x18', b'{"fb":null}\n']


def test_alarm_clear_executes_before_tid_metadata(monkeypatch):
    monkeypatch.setattr("tinyg_hil.driver.time.sleep", lambda _: None)
    alarm = True
    def answer(data):
        nonlocal alarm
        obj = json.loads(data)
        for key, value in obj.items():
            if key == "tid" and alarm: return [reply(obj["tid"], 204)]
            if key == "clear": alarm = False
        return [keyed("clear", None, obj.get("tid"))]
    tg = driver(answer=answer)
    assert tg.send_json({"clear": None}, check=True).status == 0
    assert not alarm
    assert list(json.loads(tg.transport.writes[0])) == ["clear"]


@pytest.mark.parametrize("state", [2, 12, 13])
def test_alarm_shutdown_panic_get_is_plain_and_keyed(state):
    def answer(data):
        assert data == b'{"stat":null}\n'
        return [keyed("sv", 1), keyed("stat", state)]
    tg = driver(answer=answer)
    assert tg.get("stat") == state
    assert len(tg.transport.writes) == 1


def test_missing_tid_get_can_match_by_key_but_set_stays_strict():
    tg = driver(answer=lambda data: [keyed("fb", 450.01)])
    assert tg.get("fb", timeout=0.01) == 450.01
    tg = driver(answer=lambda data: [keyed("ja", 100001)])
    with pytest.raises(NoResponse): tg.send_json({"ja": 100001}, timeout=0.01, allow="test")


def test_alarm_action_get_is_never_replayed():
    def answer(data):
        assert data == b'{"md":null}\n'
        return [keyed("md", None)]
    tg = driver(answer=answer)
    assert tg.send_json({"md": None}).status == 0
    assert len(tg.transport.writes) == 1


def test_bootstrap_low_verbosity_plain_footer_is_alive():
    tg = driver(answer=lambda data: [reply()])
    assert tg._responds()
    assert tg.transport.writes == [b'{"fb":null}\n']


@pytest.mark.parametrize("line", ['{"rt":{"fi":1}}', '{"rt":{"hb":1}}',
                                  '{"rt":{"qt":"0123456789abcdef0123456789abcdef"}}',
                                  '{"rt":null}', '{rt:{fi:1}}'])
def test_runtime_envelope_preserves_wire_and_key_matches(line):
    def answer(data):
        assert data == (line + "\n").encode()
        return [keyed("fb", 450.01), keyed("rt", {"fi": 9}, 99), keyed("rt", {"fi": 1})]
    tg = driver(answer=answer)
    assert tg.send_line(line, check=True).body["rt"]["fi"] == 1
    assert tg._next_tid == 1


def test_motion_sync_gc_is_plain_and_key_matched():
    tg = driver(answer=lambda data: [keyed("fb", 450.01), keyed("gc", "M400")])
    tg.send_json({"gc": "M400"}, allow="test")
    assert json.loads(tg.transport.writes[0]) == {"gc": "M400"}


def test_low_verbosity_get_after_jv_write_can_use_empty_reply():
    def answer(data):
        return [reply(json.loads(data).get("tid"))]
    tg = driver(answer=answer)
    tg.send_json({"jv": 1}, allow="test")
    assert tg.get("fb") is None
    assert tg.transport.writes[-1] == b'{"fb":null}\n'


def test_runtime_rejection_without_echo_is_reported_once():
    tg = driver(answer=lambda data: [reply(status=106)])
    assert tg.send_line('{"rt":{"qt":"bad"},"ja":1}').status == 106
    assert len(tg.transport.writes) == 1


@pytest.mark.parametrize("line,key,value", [
    ('{"sys":{"mt":0}}', "sys", {"mt": 0.1}),
    ('{"sr":{"posx":true,"stat":true}}', "sr", {"posx": True, "stat": True}),
    ('{"srs":{"posx":true}}', "srs", {"posx": True}),
    ('{sr:{posx:t,stat:t}}', "sr", {"posx": True, "stat": True}),
    ('{ja:100001}', "ja", 100001),
])
def test_group_and_relaxed_requests_keep_wire_and_match_untagged_keys(line, key, value):
    def answer(data):
        assert data == (line + "\n").encode()
        return [keyed("fb", 450.01), keyed(key, value, 99),
                json.dumps({"r": {key: value}, "f": [2, 0, 27]})]
    tg = driver(answer=answer)
    assert tg.send_line(line, check=True).body[key] == value
    assert tg._next_tid == 1


@pytest.mark.parametrize("line", ['{"tid":7,"fb":null}', '{"fb":null,"tid":7}',
                                  '{tid:7,fb:null}', '{sr:{posx:t},tid:7}'])
def test_caller_tid_is_preserved_and_used(line):
    tg = driver(answer=lambda data: [reply(1), keyed("fb", 450.01, 7)])
    assert tg.send_line(line).event.data["tid"] == 7
    assert tg.transport.writes == [(line + "\n").encode()]
    assert tg._next_tid == 1


@pytest.mark.parametrize("line,status,count", [('{"gc":null}', 3, 12),
                                               ('{"fv":null}', 107, 256),
                                               ('{"sys":{"mt":0}}', 104, 27),
                                               ('{"ja":100001}', 204, 27)])
def test_footer_only_untagged_error_is_returned_without_retry(line, status, count):
    def answer(data):
        return [reply(99, status), json.dumps({"r": {}, "f": [2, status, count]}),
                '{"sr":{"stat":3}}']
    tg = driver(answer=answer)
    r = tg.send_line(line)
    assert r.status == status
    assert r.event.data["f"] == [2, status, count]
    assert len(tg.transport.writes) == 1


def test_footer_only_untagged_error_still_obeys_check():
    tg = driver(answer=lambda data: ['{"r":{},"f":[2,107,256]}'])
    with pytest.raises(StatusError):
        tg.send_line('{"fv":null}', check=True)


def test_unrelated_nonempty_error_is_not_a_keyed_reply():
    tg = driver(answer=lambda data: [keyed("sv", 1, status=104), keyed("fv", 0.97)])
    assert tg.get("fv") == 0.97


@pytest.mark.parametrize("verbosity", [1, 3, 4])
def test_gc_success_can_have_no_echo_at_config_verbosity(verbosity):
    tg = driver(answer=lambda data: [reply()])
    tg._json_verbosity = verbosity
    assert tg.send_line('{"gc":"G1 X1 F500"}', check=True).status == 0
    assert tg.transport.writes == [b'{"gc":"G1 X1 F500"}\n']


def test_nested_tid_is_not_request_metadata():
    line = '{sr:{tid:7,stat:t}}'
    tg = driver(answer=lambda data: [keyed("sr", {"stat": True})])
    assert tg.send_line(line).body["sr"] == {"stat": True}
    assert tg.transport.writes == [(line + "\n").encode()]


@pytest.mark.parametrize("line", ['{"xvm":123}x', '{"ee":falsx}', '{"xvm":nonsense}',
                                  '{"xvm":1 2}', '{"gc":"G21"b}', '{"fb":null'])
def test_malformed_json_keeps_exact_bytes_and_matches_its_error_echo(line):
    def answer(data):
        assert data == (line + "\n").encode()
        return [json.dumps({"r": {"err": "an earlier malformed request"}, "f": [2, 111, 12]}),
                json.dumps({"r": {"err": line}, "f": [2, 111, len(data)]})]
    tg = driver(answer=answer)
    r = tg.send_line(line)
    assert r.status == 111
    assert r.body == {"err": line}
    assert tg._next_tid == 1


def test_gc_message_reply_without_gc_echo_keeps_tab_escape(monkeypatch):
    monkeypatch.setattr("tinyg_hil.driver.time.sleep", lambda _: None)
    line = json.dumps({"gc": "(msgA\tB)"}, separators=(",", ":"))
    def answer(data):
        assert data == (line + "\n").encode()
        return [keyed("fb", 450.01), json.dumps({"r": {"msg": "A\tB"}, "f": [2, 0, len(data)]})]
    tg = driver(answer=answer)
    r = tg.send_json({"gc": "(msgA\tB)"}, allow="test")
    assert r.status == 0 and r.body == {"msg": "A\tB"}
    assert tg._next_tid == 1
