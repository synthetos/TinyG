"""Harness self-tests; no board needed."""
import pytest

from tinyg_hil import dialect, protocol
from tinyg_hil.safety import Guard, MotionBlocked, gcode_risks

# Lines captured from a v8 board running 440.20
FB = '{"r":{"fb":440.20},"f":[1,0,12,1645]}'
M115 = '{"r":{},"f":[1,132,5,6965]}'
BANNER_440 = ('{"r":{"fv":0.970,"fb":440.20,"hp":1,"hv":8,"id":"0BENCH-ONE","msg":"SYSTEM READY"},'
              '"f":[1,0,0,8416]}')


@pytest.mark.parametrize("line, expected", [(FB, 1645), (M115, 6965), (BANNER_440, 8416)])
def test_checksum_440(line, expected):
    assert protocol.checksum_440(line) == expected


def test_parse_response_and_footer():
    ev = protocol.parse_line(FB, 0.0)
    assert ev.kind == "response" and ev.body == {"fb": 440.20}
    f = protocol.Footer.from_event(ev)
    assert (f.rev, f.status, f.count, f.checksum_ok) == (1, 0, 12, True)


def test_parse_footer_rev3():
    f = protocol.Footer.from_event(protocol.parse_line('{"r":{"fb":449.01},"f":[3,0,11]}', 0.0))
    assert (f.rev, f.status, f.count, f.checksum_ok) == (3, 0, 11, None)


def test_parse_banners():
    assert protocol.parse_line(BANNER_440, 0.0).kind == "banner"
    ev = protocol.parse_line('{sr:{fv:0.970,fb:449.01,msg:"SYSTEM READY"}}', 0.0)   # relaxed syntax
    assert ev.kind == "banner" and ev.body["fb"] == 449.01


@pytest.mark.parametrize("line, kind", [
    ('{"sr":{"stat":3}}', "status"), ('{"qr":32}', "queue"), ('{"rx":254}', "rx"),
    ('{"er":{"st":29}}', "exception"), ('{"ack":true}', "ack"), ("tinyg [mm] ok> ", "prompt"),
    ("[fb]  firmware build            440.20", "text"), ('{"r":{"fb":', "malformed"),
])
def test_classify(line, kind):
    assert protocol.parse_line(line, 0.0).kind == kind


@pytest.mark.parametrize("line", ["G1 X1", "X1", "G0 Z-1", "G28", "G28.2 X0", "G38.2 Z-1 F100", "M3 S1000",
                                  "M8", "G53 G0 X0", "g1x1"])
def test_guard_blocks(line):
    assert gcode_risks(line)
    with pytest.raises(MotionBlocked):
        Guard().check_gcode(line)


@pytest.mark.parametrize("line", ["G21", "G10 L2 P2 X12.5", "G92 X0 Y0", "G28.1", "G28.3 X0", "G43.1 Z5",
                                  "M5", "M9", "M2", "G1 F100", "(X1 in a comment)", "G4 P0.5", "M115"])
def test_guard_allows(line):
    assert not gcode_risks(line)
    Guard().check_gcode(line)


def test_guard_json():
    g = Guard()
    g.check_json({"fb": None})
    g.check_json({"ex": None})
    for obj in ({"gc": "G1 X1"}, {"home": 1}, {"ex": 0}, {"baud": 5}):
        with pytest.raises(MotionBlocked):
            g.check_json(obj)
    g.check_json({"ex": 0}, allow="flow control test")


@pytest.mark.parametrize("name, fb", [("d440", 440.21), ("d449", 449.01)])
def test_tables(name, fb):
    d = dialect.load(name)
    assert d.table["fb"] == fb
    assert d.status("UNRECOGNIZED_NAME") == 100
    assert d.status("MCODE_COMMAND_UNSUPPORTED") == 132
    assert "xvm" in d.persisted_tokens() and "fb" in d.tokens
    assert d.stat("SHUTDOWN") == (11 if name == "d440" else 12)



@pytest.mark.parametrize("raw, kind, body", [
    ('\x00{"r":{"1ma":0},"f":[3,0,10]}', "response", {"1ma": 0}),          # leading NUL: salvaged
    ('\x00\x00{"sr":{"stat":4}}', "status", None),                          # several leading NULs
])
def test_leading_nul_is_salvaged_and_marked(raw, kind, body):
    ev = protocol.parse_line(raw, 0.0)
    assert ev.kind == kind and ev.damaged == raw and not ev.raw.startswith("\x00")
    if body is not None:
        assert ev.body == body


@pytest.mark.parametrize("raw", [
    '{"r":{"x":1\x002},"f":[3,0,10]}',             # embedded in a number: never {"x":12}
    '{"r":{"msg":"a\x00b"},"f":[3,0,10]}',         # embedded in a string
    '\x00{"r":{"x":1\x002},"f":[3,0,10]}',         # leading and embedded
])
def test_embedded_nul_stays_malformed(raw):
    ev = protocol.parse_line(raw, 0.0)
    assert ev.kind == "malformed"


def test_salvaged_nul_line_is_recorded():
    """The run record keeps the line as received and counts it as salvaged."""
    from types import SimpleNamespace
    from tinyg_hil.record import RunRecorder
    ev = protocol.parse_line('\x00{"r":{"1ma":0},"f":[3,0,10]}', 1.0)
    tg = SimpleNamespace(transport=SimpleNamespace(sent=[], history=[ev]))
    rec = RunRecorder.__new__(RunRecorder)
    rec.pending = {"n": {"nodeid": "n", "t0": 0.0, "sent_start": 0, "history_start": 0}}
    rec.end("n", tg)
    out = rec.pending["n"]
    assert out["salvaged_lines"] == 1 and out["events"][0]["damaged"].startswith("\x00")
