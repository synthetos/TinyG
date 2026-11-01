"""Switchbox client and spec, without hardware: a fake serial port stands in for the board."""
import io
import json
import os
import pathlib
import re
import sys
import time

import pytest

from tinyg_hil.lock import BoardBusy, BoardLock
from tinyg_hil.switchbox import (
    SPEC_PATH,
    SwitchBox,
    SwitchBoxError,
    SwitchBoxRebooted,
    SwitchBoxSpecMismatch,
    SwitchBoxTimeout,
    decode_mask,
    load_spec,
    operation_ids,
    prepared,
    redact,
    redact_text,
    refuse_port,
    resolve_port,
    session_box,
    spec_digest,
    spec_mismatch,
    vendored_from,
)
from tinyg_hil.switchbox import main as switchbox_cli

HIL = pathlib.Path(__file__).resolve().parents[1]
ROM_BANNER = b"\x00\xe0\x1c\x8c\xff ets Jan  8 2013,rst cause:2, boot mode:(3,6)\r\n\xf2\x8c\x00\n"


class FakeBoard:
    """Unopened-serial stand-in. Records how it was set up; answers each request line via handlers."""

    def __init__(self, handlers=None, uptime_ms=600000, on_open=b"", trickle=False):
        self.log = []                   # ("set", attr, value) and ("open",), in order
        self.written = []               # request objects
        self.out = bytearray()
        self.is_open = False
        self.trickle = trickle          # read() hands out one byte at a time
        self.on_open = on_open
        self.uptime_ms = uptime_ms
        self.handlers = {"getInfo": lambda req: self.ok(req, self.info()),
                         "releaseAll": lambda req: self.ok(req, {"changed": 0}),
                         "idleAll": lambda req: self.ok(req, {"changed": 0})}
        self.handlers.update(handlers or {})

    def __setattr__(self, name, value):
        if name in ("port", "baudrate", "timeout", "write_timeout", "dtr", "rts"):
            self.__dict__.setdefault("log", []).append(("set", name, value))
        object.__setattr__(self, name, value)

    def info(self):
        return {"fw_version": "0.1.0", "api_version": "1.0.0", "uptime_ms": self.uptime_ms, "boot_id": "cafe0001",
                "events": {"head": 1, "oldest": 1, "capacity": 128}, "spec_sha256": spec_digest(load_spec())}

    @staticmethod
    def ok(req, result):
        return [{"id": req["id"], "ok": True, "result": result}]

    @staticmethod
    def error(req, code, msg="bad"):
        return [{"id": req["id"], "ok": False, "error": {"code": code, "msg": msg}}]

    def open(self):
        self.log.append(("open",))
        self.is_open = True
        self.out += self.on_open

    def close(self):
        self.is_open = False

    @property
    def in_waiting(self):
        return len(self.out)

    def read(self, n):
        if not self.out:
            time.sleep(0.001)
            return b""
        n = 1 if self.trickle else n
        chunk = bytes(self.out[:n])
        del self.out[:n]
        return chunk

    def write(self, data):
        for line in data.decode().splitlines():
            req = json.loads(line)
            self.written.append(req)
            handler = self.handlers.get(req["op"])
            for frame in handler(req) if handler else []:
                self.out += frame if isinstance(frame, bytes) else json.dumps(frame).encode() + b"\n"
        return len(data)


NO_BENCHES = {}                         # offline tests never read the machine's bench file


def connect(board, **kw):
    kw.setdefault("lock", False)
    kw.setdefault("benches", NO_BENCHES)
    return SwitchBox("/dev/cu.fake-switchbox", serial_factory=lambda: board, timeout=0.3, **kw).connect()


def sent(board, op_name):
    return [{k: v for k, v in r.items() if k != "id"} for r in board.written if r["op"] == op_name]


# ---- connection --------------------------------------------------------------------------------

def lines_log(board):
    opened = board.log.index(("open",))
    before = {(e[1], e[2]) for e in board.log[:opened] if e[0] == "set"}
    after = [(e[1], e[2]) for e in board.log[opened:] if e[0] == "set" and e[1] in ("dtr", "rts")]
    return before, after


def test_connect_never_leaves_rts_asserted_alone():
    """Default: both asserted before open (as macOS opens the port), then RTS dropped before DTR."""
    board = FakeBoard()
    connect(board)
    before, after = lines_log(board)
    assert {("dtr", True), ("rts", True), ("port", "/dev/cu.fake-switchbox"), ("baudrate", 115200)} <= before
    assert after == [("rts", False), ("dtr", False)]


def test_other_line_strategies():
    for lines, level in (("deasserted", False), ("asserted", True)):
        board = FakeBoard()
        connect(board, lines=lines)
        before, after = lines_log(board)
        assert {("dtr", level), ("rts", level)} <= before and after == []
    with pytest.raises(ValueError):
        SwitchBox("/dev/cu.fake-switchbox", lines="toggle")


def test_rom_banner_and_non_json_lines_are_skipped():
    board = FakeBoard(on_open=ROM_BANNER + b"hello\n[1,2]\n{broken\n")
    sb = connect(board)
    assert sb.info_at_connect["boot_id"] == "cafe0001"
    assert sb.garbage_count >= 4
    assert sb.reset_on_connect is False


def test_reset_on_connect_is_detected():
    boot = json.dumps({"event": "boot", "seq": 1, "t_us": 90000, "src": "boot"}).encode() + b"\n"
    with pytest.raises(SwitchBoxRebooted):
        connect(FakeBoard(on_open=ROM_BANNER + boot))
    with pytest.raises(SwitchBoxRebooted):
        connect(FakeBoard(uptime_ms=40))                  # booted after the port opened
    sb = connect(FakeBoard(uptime_ms=40), allow_reset=True)
    assert sb.reset_on_connect is True


def test_connect_retries_getinfo_while_the_board_boots():
    board = FakeBoard()
    real = board.handlers["getInfo"]
    calls = []
    board.handlers["getInfo"] = lambda req: (calls.append(req["id"]), [] if len(calls) == 1 else real(req))[1]
    sb = connect(board)
    assert len(calls) == 2 and sb.info_at_connect["uptime_ms"] == 600000


def test_connect_takes_the_board_lock():
    port = f"/dev/cu.fake-switchbox-{os.getpid()}"
    try:
        first = SwitchBox(port, serial_factory=FakeBoard, timeout=0.3, benches=NO_BENCHES).connect()
        try:
            with pytest.raises(BoardBusy):
                SwitchBox(port, serial_factory=FakeBoard, timeout=0.3, lock_wait=0, benches=NO_BENCHES).connect()
        finally:
            first.disconnect()
        SwitchBox(port, serial_factory=FakeBoard, timeout=0.3, benches=NO_BENCHES).connect().disconnect()
    finally:
        BoardLock(port).path.unlink(missing_ok=True)


BENCHES = {"description": "synthetic",
           "never_open": ["/dev/cu.usbserial-OTHER"],
           "switchbox": {"port": "/dev/cu.usbserial-SWITCHBOX"},
           "/dev/cu.usbserial-BENCH1": {"name": "bench-1", "switchbox": {"port": "/dev/cu.usbserial-SB1"}},
           "/dev/cu.usbserial-BENCH2": {"name": "bench-2", "switchbox": None}}


def test_tinyg_and_never_open_ports_are_refused():
    for port in ("/dev/cu.usbserial-BENCH1", "/dev/cu.usbserial-BENCH2", "/dev/cu.usbserial-OTHER"):
        with pytest.raises(ValueError):
            refuse_port(port, BENCHES)
        with pytest.raises(ValueError):
            SwitchBox(port, serial_factory=FakeBoard, lock=False, benches=BENCHES).connect()
    refuse_port("/dev/cu.usbserial-SWITCHBOX", BENCHES)
    refuse_port("/dev/cu.usbserial-0001", BENCHES)


def test_example_bench_file_documents_the_switchbox_entries():
    example = json.loads((HIL / "config" / "benches.json").read_text())
    assert isinstance(example["never_open"], list) and "port" in example["switchbox"]
    assert all("switchbox" in v for k, v in example.items() if k.startswith("/dev/"))


def test_port_resolution():
    assert resolve_port("/dev/x", "/dev/cu.usbserial-BENCH1", BENCHES) == "/dev/x"
    assert resolve_port(None, "/dev/cu.usbserial-BENCH1", BENCHES) == "/dev/cu.usbserial-SB1"
    assert resolve_port(None, "/dev/cu.usbserial-BENCH2", BENCHES) is None    # bench without a switchbox
    assert resolve_port(None, None, BENCHES) == "/dev/cu.usbserial-SWITCHBOX"  # standalone run
    assert resolve_port(None, None, {}) is None


# ---- framing -----------------------------------------------------------------------------------

def test_reply_matched_by_id_with_stale_replies_and_events_in_between():
    def state(req):
        return [{"id": req["id"] - 1, "ok": True, "result": {"late": True}},
                {"event": "edge", "seq": 7, "ch": "xmin", "state": "closed"},
                b"\xff\xfe garbage\n",
                {"id": req["id"], "ok": True, "result": {"channels": []}}]
    sb = connect(FakeBoard({"getState": state}))
    assert sb.state() == {"channels": []}
    assert sb.stale[-1]["result"] == {"late": True}
    assert sb.received[-1]["seq"] == 7


def test_partial_lines_are_reassembled():
    sb = connect(FakeBoard({"getConfig": lambda req: FakeBoard.ok(req, {"channels": [{"name": "xmin"}]})},
                           trickle=True))
    assert sb.config()["channels"][0]["name"] == "xmin"


def test_error_reply_raises_with_its_code():
    sb = connect(FakeBoard({"setChannel": lambda req: FakeBoard.error(req, "unknown_channel", "unknown channel 'wmin'")}))
    with pytest.raises(SwitchBoxError) as e:
        sb.close("w", "min")
    assert e.value.code == "unknown_channel" and "wmin" in e.value.msg


def test_unanswered_request_times_out():
    sb = connect(FakeBoard())
    with pytest.raises(SwitchBoxTimeout):
        sb.call("getState", timeout=0.05)


def test_overlong_request_is_refused_before_sending():
    board = FakeBoard()
    sb = connect(board)
    with pytest.raises(ValueError):
        sb.set_config([{"name": "x" * 200, "gpio": 4}] * 8)
    assert not sent(board, "setConfig")


# ---- operations --------------------------------------------------------------------------------

def test_soft_switch_methods_map_to_set_channel():
    board = FakeBoard({"setChannel": lambda req: FakeBoard.ok(req, {"changed": True})})
    sb = connect(board)
    sb.close("x", "min", ttl_ms=500)
    sb.open("x", "min")
    sb.trip("ymax")
    sb.release("y", "max")
    sb.contact("z", "max", state="open")
    sb.contact("zmin", ttl_ms=0)
    assert sent(board, "setChannel") == [
        {"op": "setChannel", "name": "xmin", "action": "trip", "ttl_ms": 500},
        {"op": "setChannel", "name": "xmin", "action": "release"},
        {"op": "setChannel", "name": "ymax", "action": "trip"},
        {"op": "setChannel", "name": "ymax", "action": "release"},
        {"op": "setChannel", "name": "zmax", "state": "open"},
        {"op": "setChannel", "name": "zmin", "state": "closed", "ttl_ms": 0},
    ]


def test_pulse_waits_until_the_board_reports_it_done():
    pending = iter([{"kind": "pulse", "until_us": 3000}, None])

    def state(req):
        return FakeBoard.ok(req, {"channels": [{"name": "xmin", "pending": next(pending)}]})
    board = FakeBoard({"pulseChannel": lambda req: FakeBoard.ok(req, {"start_us": 1000, "end_us": 3000}),
                       "getState": state})
    sb = connect(board)
    t0 = time.monotonic()
    sb.pulse("x", "min", 2)
    assert time.monotonic() - t0 >= 0.002
    assert sent(board, "pulseChannel") == [{"op": "pulseChannel", "name": "xmin", "ms": 2, "action": "trip"}]
    assert len(sent(board, "getState")) == 2


def test_bounce_defaults_to_a_trip_and_can_skip_waiting():
    board = FakeBoard({"bounceChannel": lambda req: FakeBoard.ok(req, {"start_us": 0, "end_us": 10**9})})
    sb = connect(board)
    sb.bounce("z", "min", edges=6, interval_us=300, wait=False)
    sb.bounce("zmin", edges=2, interval_us=50, final_state="open", ttl_ms=100, wait=False)
    assert sent(board, "bounceChannel") == [
        {"op": "bounceChannel", "name": "zmin", "edges": 6, "interval_us": 300, "final_action": "trip"},
        {"op": "bounceChannel", "name": "zmin", "edges": 2, "interval_us": 50, "final_state": "open", "ttl_ms": 100},
    ]


def test_events_pages_until_the_newest():
    pages = {0: ([1, 2], 3, True), 3: ([3], 4, False)}

    def events(req):
        seqs, nxt, more = pages[req.get("since", 0)]
        return FakeBoard.ok(req, {"events": [{"seq": s} for s in seqs], "next": nxt, "more": more})
    sb = connect(FakeBoard({"getEvents": events}))
    assert [e["seq"] for e in sb.events(all=True)] == [1, 2, 3]
    assert [e["seq"] for e in sb.events()] == [1, 2]


def test_wifi_credentials_are_redacted_in_traffic_and_records():
    board = FakeBoard({"setWifi": lambda req: FakeBoard.ok(req, {"ssid": req["ssid"], "cleared": False}),
                       "getState": lambda req: FakeBoard.ok(req, {"wifi": {"ssid": "examplenet", "mode": "sta"}})})
    sb = connect(board)
    sb.set_wifi("examplenet", "hunter2hunter2")
    assert board.written[-1]["password"] == "hunter2hunter2"
    assert sb.state()["wifi"]["ssid"] == "examplenet"            # the caller still sees the reply as sent
    logged = json.dumps(list(sb.traffic))
    assert "hunter2" not in logged and "examplenet" not in logged
    assert redact({"wifi": {"ssid": "examplenet", "ap": {"ssid": "tinyg-switchbox-ABCD"}}}) == \
        {"wifi": {"ssid": "***", "ap": {"ssid": "tinyg-switchbox-ABCD"}}}


def test_cli_reads_the_wifi_password_from_stdin(monkeypatch, capsys):
    board = FakeBoard({"setWifi": lambda req: FakeBoard.ok(req, {"ssid": req["ssid"], "cleared": False,
                                                                 "reconnecting": True})})
    monkeypatch.setattr(sys, "stdin", io.StringIO("placeholder-pw\n"))
    switchbox_cli(["wifi", "--port", "/dev/cu.fake-switchbox", "--ssid", "example-net", "--password-stdin"],
                  serial_factory=lambda: board, lock=False, timeout=0.3)
    assert board.written[-1] == {"id": board.written[-1]["id"], "op": "setWifi", "ssid": "example-net",
                                 "password": "placeholder-pw"}
    out = capsys.readouterr().out
    assert "placeholder-pw" not in out and "example-net" not in out


def scan_board(*ssids):
    nets = [{"ssid": s, "rssi": -50, "channel": 6, "encryption": "wpa2", "hidden": False} for s in ssids]
    return FakeBoard({"scanWifi": lambda req: FakeBoard.ok(req, {"networks": nets, "count": len(nets)}),
                      "setWifi": lambda req: FakeBoard.ok(req, {"ssid": req["ssid"], "cleared": False,
                                                                "reconnecting": True})})


def test_match_ssid_needs_exactly_one_distinct_name():
    sb = connect(scan_board("ExampleNet", "ExampleNet", "other"))     # one name from two access points
    assert sb.match_ssid(" examplenet ") == "ExampleNet"
    for ssids in (("other",), ("ExampleNet", "examplenet ")):
        with pytest.raises(LookupError) as e:
            connect(scan_board(*ssids)).match_ssid("examplenet")
        assert "ExampleNet" not in str(e.value)
    assert "ExampleNet" not in json.dumps(list(sb.traffic))


def test_cli_wifi_match_ignore_case(monkeypatch, capsys):
    board = scan_board("ExampleNet", "other")
    monkeypatch.setattr(sys, "stdin", io.StringIO("placeholder-pw\n"))
    switchbox_cli(["wifi", "--port", "/dev/cu.fake-switchbox", "--ssid", "examplenet", "--password-stdin",
                   "--match-ignore-case"], serial_factory=lambda: board, lock=False, timeout=0.3)
    assert sent(board, "setWifi") == [{"op": "setWifi", "ssid": "ExampleNet", "password": "placeholder-pw"}]
    out = json.loads(capsys.readouterr().out)
    assert out == {"ssid": "***", "cleared": False, "reconnecting": True, "matched": "ignoring case/spaces"}
    board = scan_board("ExampleNet", "EXAMPLENET")
    monkeypatch.setattr(sys, "stdin", io.StringIO("placeholder-pw\n"))
    with pytest.raises(LookupError):
        switchbox_cli(["wifi", "--port", "/dev/cu.fake-switchbox", "--ssid", "examplenet", "--password-stdin",
                       "--match-ignore-case"], serial_factory=lambda: board, lock=False, timeout=0.3)
    assert not sent(board, "setWifi")


def test_decode_mask():
    assert decode_mask("0110", ["xmin", "xmax", "ymin", "ymax"]) == {"xmin": 0, "xmax": 1, "ymin": 1, "ymax": 0}


# ---- spec --------------------------------------------------------------------------------------

def client_ops():
    return {getattr(SwitchBox, n).sb_op: n for n in dir(SwitchBox) if hasattr(getattr(SwitchBox, n), "sb_op")}


def test_spec_is_openapi_31_and_every_operation_has_a_client_method():
    spec = load_spec()
    assert spec["openapi"].startswith("3.1")
    ops = operation_ids(spec)
    assert ops == set(client_ops()), f"spec only: {ops - set(client_ops())}; client only: {set(client_ops()) - ops}"
    for path in spec["paths"].values():
        for o in path.values():
            assert o["summary"] and o["description"] and "200" in o["responses"]
            assert set(o["x-transports"]) <= {"serial", "http", "ws"}


def test_spec_refs_resolve():
    spec = load_spec()

    def walk(node):
        if isinstance(node, dict):
            for k, v in node.items():
                if k == "$ref":
                    target = spec
                    for part in v.removeprefix("#/").split("/"):
                        target = target[part]
                else:
                    walk(v)
        elif isinstance(node, list):
            for v in node:
                walk(v)
    walk(spec)


def test_vendored_spec_names_its_switchbox_commit():
    """The pinned copy says where it came from; load_spec() returns exactly the switchbox's document."""
    src = vendored_from()
    assert src["repository"] == "synthetos/tinyg-switchbox" and re.fullmatch(r"[0-9a-f]{40}", src["commit"])
    raw = json.loads(SPEC_PATH.read_text())
    spec = load_spec()
    assert "x-vendored-from" in raw and "x-vendored-from" not in spec
    assert {k: v for k, v in raw.items() if k != "x-vendored-from"} == spec


def test_spec_mismatch_explains_which_side_to_update():
    spec = load_spec()
    ok = {"api_version": spec["info"]["version"], "spec_sha256": spec_digest(spec)}
    assert spec_mismatch(ok) is None
    other = spec_mismatch({"api_version": "9.9.9", "spec_sha256": "0" * 64})
    assert "different API version" in other and "API 9.9.9" in other and vendored_from()["commit"][:12] in other
    same_version = spec_mismatch({"api_version": spec["info"]["version"], "spec_sha256": "f" * 64})
    assert "same API version" in same_version and "switchbox-openapi.json" in same_version


# ---- credentials in logs (review follow-up) -----------------------------------------------------

def test_passwords_are_always_masked_and_only_an_ssid_may_be_the_access_point_name():
    ap = "tinyg-switchbox-ABCD"
    assert redact({"password": ap, "ssid": ap, "sta": {"ssid": "examplenet", "password": 12345}}) == \
        {"password": "***", "ssid": ap, "sta": {"ssid": "***", "password": "***"}}
    text = redact_text(json.dumps({"ssid": ap, "password": ap + "-secret", "other": {"ssid": "examplenet"}}))
    assert "secret" not in text and "examplenet" not in text and f'"ssid": "{ap}"' in text
    assert redact_text('{"password": null, "ssid": 7}') == '{"password":"***", "ssid":"***"}'


def test_cut_off_values_are_masked_to_the_end():
    for line in ('{"id":1,"password":"examplepw', '{"ssid": "examplenet\\"more', '{"ssid":"tinyg-switchbox-AB'):
        assert "example" not in redact_text(line) and "AB" not in redact_text(line)


def test_lines_are_redacted_before_they_are_shortened():
    """A line cut inside a credential must not leave its first characters in the traffic log."""
    def event_line(limit, key="ssid"):
        head = '{"event": "wifi", "pad": "'
        tail = f'", "{key}": "examplenet-private"}}'
        pad = limit - len(head) - len(f'", "{key}": "exam')
        return (head + "x" * pad + tail).encode() + b"\n"
    garbage = b'not json "password": "examplepw-private" ' + b"y" * 300 + b"\n"
    cut = b'not json ' + b"z" * (200 - len('not json "password": "exam')) + b'"password": "examplepw-private"\n'
    reply = lambda req: [event_line(400), event_line(400, "password"), garbage, cut] + FakeBoard.ok(req, {})
    sb = connect(FakeBoard({"getState": reply}))
    sb.state()
    logged = json.dumps(list(sb.traffic))
    assert "exam" not in logged, logged
    assert any(d == "ev" and len(s) == 400 for _, d, s in sb.traffic)


# ---- cleanup when connecting or preparing fails -------------------------------------------------

class LineFailBoard(FakeBoard):
    """Raises when connect() drops RTS after opening, like a USB adapter vanishing mid-setup."""

    def __setattr__(self, name, value):
        if name == "rts" and value is False and self.__dict__.get("is_open"):
            raise OSError("adapter went away")
        super().__setattr__(name, value)


class CloseFailBoard(FakeBoard):
    def close(self):
        super().close()
        raise OSError("close failed")


def lock_port(tag):
    return f"/dev/cu.fake-switchbox-{tag}-{os.getpid()}"


def test_a_failed_line_setup_closes_the_port_and_releases_the_lock():
    port, board = lock_port("lines"), LineFailBoard()
    try:
        with pytest.raises(OSError, match="adapter went away"):
            SwitchBox(port, serial_factory=lambda: board, timeout=0.3, benches=NO_BENCHES).connect()
        assert ("open",) in board.log and not board.is_open          # it was opened, then closed
        BoardLock(port).acquire("after a failed connect", wait=0).release()
    finally:
        BoardLock(port).path.unlink(missing_ok=True)


def test_disconnect_releases_the_lock_even_if_close_fails():
    port = lock_port("close")
    try:
        sb = SwitchBox(port, serial_factory=CloseFailBoard, timeout=0.3, benches=NO_BENCHES).connect()
        with pytest.raises(OSError, match="close failed"):
            sb.disconnect()
        assert sb.ser is None
        BoardLock(port).acquire("after a failed close", wait=0).release()
    finally:
        BoardLock(port).path.unlink(missing_ok=True)


def session_kwargs(board):
    return {"serial_factory": lambda: board, "timeout": 0.3, "lock": False, "benches": NO_BENCHES}


def test_session_box_refuses_a_box_with_another_spec_before_touching_contacts():
    board = FakeBoard()
    board.info = lambda: {**FakeBoard.info(board), "api_version": "9.9.9", "spec_sha256": "0" * 64}
    seen = []
    with pytest.raises(SwitchBoxSpecMismatch, match="API 9.9.9"):
        with session_box("/dev/cu.fake-switchbox", identity=seen.append, **session_kwargs(board)):
            pytest.fail("a mismatched box was handed to a test")
    assert seen and seen[0]["api_version"] == "9.9.9"            # identity still recorded
    assert {r["op"] for r in board.written} == {"getInfo"} and not board.is_open


def test_session_box_cleans_up_when_recording_the_identity_fails():
    port, board = lock_port("ident"), FakeBoard()

    def broken(_):
        raise RuntimeError("recorder failed")
    try:
        with pytest.raises(RuntimeError, match="recorder failed"):
            with session_box(port, identity=broken, serial_factory=lambda: board, timeout=0.3, benches=NO_BENCHES):
                pass
        assert {r["op"] for r in board.written} == {"getInfo"} and not board.is_open
        BoardLock(port).acquire("after a failed session setup", wait=0).release()
    finally:
        BoardLock(port).path.unlink(missing_ok=True)


def test_session_box_releases_everything_at_the_end():
    board = FakeBoard()
    with session_box("/dev/cu.fake-switchbox", **session_kwargs(board)) as sb:
        assert sb.ser is board
    assert [r["op"] for r in board.written] == ["getInfo", "releaseAll"] and not board.is_open


def events_board(**handlers):
    page = lambda req: FakeBoard.ok(req, {"events": [], "head": 5, "next": 5, "more": False})
    return FakeBoard({"getEvents": page, **handlers})


def test_prepared_releases_contacts_when_its_own_setup_fails():
    board = events_board(idleAll=lambda req: FakeBoard.error(req, "busy"))
    sb, records = connect(board), []
    with pytest.raises(SwitchBoxError, match="busy"):
        with prepared(sb, lambda *a: records.append(a)):
            pytest.fail("setup failed but the test ran")
    assert [r["op"] for r in board.written[1:]] == ["releaseAll", "idleAll", "releaseAll"]
    assert records == []                                          # no mark yet, so nothing to record


def test_prepared_records_traffic_and_releases_after_the_test():
    board = events_board()
    sb, records = connect(board), []
    with pytest.raises(AssertionError):
        with prepared(sb, lambda *a: records.append(a)):
            raise AssertionError("the test failed")
    assert [r["op"] for r in board.written[1:]] == ["releaseAll", "idleAll", "getEvents", "releaseAll", "getEvents"]
    assert records and records[0][0] == "switchbox" and records[0][1]["traffic"]
