"""Switchbox client and spec, without hardware: a fake serial port stands in for the board."""
import io
import json
import os
import re
import shutil
import subprocess
import sys
import time

import pytest

from tinyg_hil.lock import BoardBusy, BoardLock
from tinyg_hil.switchbox import (
    SPEC_PATH,
    SwitchBox,
    SwitchBoxError,
    SwitchBoxRebooted,
    SwitchBoxTimeout,
    decode_mask,
    load_spec,
    operation_ids,
    redact,
    refuse_port,
    resolve_port,
    spec_digest,
)
from tinyg_hil.switchbox import main as switchbox_cli

FIRMWARE = SPEC_PATH.parent / "firmware"
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
    example = json.loads((SPEC_PATH.parent.parent / "config" / "benches.json").read_text())
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
    assert "placeholder-pw" not in capsys.readouterr().out


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


def test_spec_limits_match_the_firmware_core():
    header = (FIRMWARE / "lib/sbcore/sbcore.h").read_text()

    def const(name):
        return int(re.search(rf"\b{name}\s*=\s*(\d+)", header).group(1))
    limits = load_spec()["x-limits"]
    assert limits["channels"] == const("MAX_CHANNELS")
    assert limits["event_ring"] == const("EVENT_RING")
    assert limits["events_page_max"] == const("EVENTS_PAGE_MAX")
    assert limits["ttl_ms_max"] == const("TTL_MS_MAX")
    assert limits["pulse_ms_max"] == const("PULSE_MS_MAX")
    assert limits["pulse_us_max"] == const("PULSE_US_MAX")
    assert limits["bounce_edges_max"] == const("BOUNCE_EDGES_MAX")
    assert limits["bounce_interval_us"] == [const("BOUNCE_INTERVAL_US_MIN"), const("BOUNCE_INTERVAL_US_MAX")]
    assert limits["inline_us"] == const("INLINE_US")


def test_firmware_embeds_exactly_this_spec(tmp_path):
    """Runs the build's embed step on a copy: the board serves this file, minified, with this digest."""
    fw = tmp_path / "firmware"
    (fw / "scripts").mkdir(parents=True)
    (fw / "ui").mkdir()
    shutil.copy(FIRMWARE / "scripts/embed_assets.py", fw / "scripts")
    shutil.copy(FIRMWARE / "ui/index.html", fw / "ui")
    shutil.copy(SPEC_PATH, tmp_path / "openapi.json")
    subprocess.run([sys.executable, "-I", str(fw / "scripts/embed_assets.py")], check=True, capture_output=True)
    header = (fw / "src/generated/assets.h").read_text()
    spec = load_spec()
    assert re.search(r'SB_SPEC_SHA256 "([0-9a-f]{64})"', header).group(1) == spec_digest(spec)
    assert re.search(r'SB_API_VERSION "([^"]+)"', header).group(1) == spec["info"]["version"]
    data = bytes(int(b, 16) for b in re.findall(r"0x([0-9a-f]{2})",
                                                header.split("SPEC_JSON[]")[1].split("};")[0]))
    assert b"\n" not in data and json.loads(data) == spec       # one serial line, same document
