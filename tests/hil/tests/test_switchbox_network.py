"""Switchbox over the network: REST, WebSocket, /openapi.json, the web UI and mDNS, from this host.

Needs --switchbox and the board in WiFi station mode (it learns the IP over serial). Skips when the
board is only on its own access point. The WebSocket client is a minimal RFC 6455 one (stdlib only).
"""
import base64
import hashlib
import json
import os
import re
import socket
import time
import urllib.error
import urllib.request

import pytest

from tinyg_hil.switchbox import load_spec, redact

pytestmark = pytest.mark.switchbox
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


@pytest.fixture
def host(switchbox):
    wifi = redact(switchbox.info())["wifi"]
    if wifi.get("mode") != "sta" or not wifi.get("ip"):
        pytest.skip("switchbox is not on WiFi in station mode")
    return wifi["ip"]


def http(method, url, body=None, timeout=8):
    data = json.dumps(body).encode() if body is not None and not isinstance(body, bytes) else body
    req = urllib.request.Request(url, data=data, method=method, headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, r.headers.get("Content-Type", ""), r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.headers.get("Content-Type", ""), e.read()


def http_json(method, url, body=None):
    """Status and parsed JSON body, with WiFi SSIDs masked: a failing assert prints values into the record."""
    status, ctype, raw = http(method, url, body)
    assert ctype.startswith("application/json"), (status, ctype)
    return status, redact(json.loads(raw))


class Ws:
    """Just enough RFC 6455 for text frames: handshake, masked sends, unmasked receives, ping/pong."""

    def __init__(self, host, path="/ws", timeout=8):
        self.sock = socket.create_connection((host, 80), timeout=timeout)
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall((f"GET {path} HTTP/1.1\r\nHost: {host}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                           f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
        head = b""
        while b"\r\n\r\n" not in head:
            chunk = self.sock.recv(1024)
            if not chunk:
                raise ConnectionError("closed during the handshake")
            head += chunk
        head, self.buf = head.split(b"\r\n\r\n", 1)
        assert b" 101 " in head.split(b"\r\n")[0], head
        accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
        assert accept.encode() in head, head

    def _take(self, n):
        while len(self.buf) < n:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("WebSocket closed")
            self.buf += chunk
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def _frame(self, opcode, payload):
        mask = os.urandom(4)
        n = len(payload)
        head = bytes([0x80 | opcode]) + (bytes([0x80 | n]) if n < 126 else bytes([0x80 | 126]) + n.to_bytes(2, "big"))
        self.sock.sendall(head + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(payload)))

    def send(self, obj):
        self._frame(0x1, json.dumps(obj).encode())

    def recv(self):
        while True:
            b0, b1 = self._take(2)
            n = b1 & 0x7F
            if n == 126:
                n = int.from_bytes(self._take(2), "big")
            elif n == 127:
                n = int.from_bytes(self._take(8), "big")
            payload = self._take(n)
            opcode = b0 & 0x0F
            if opcode == 0x1:
                return json.loads(payload)
            if opcode == 0x9:
                self._frame(0xA, payload)
            elif opcode == 0x8:
                raise ConnectionError("WebSocket closed by the board")

    def wait_for(self, pred, timeout=5.0):
        seen, deadline = [], time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = redact(self.recv())
            seen.append(msg)
            if pred(msg):
                return msg, seen
        raise AssertionError(f"no matching WebSocket message; got {seen[-5:]}")

    def close(self):
        try:
            self._frame(0x8, b"")
        finally:
            self.sock.close()


def test_mdns_name_resolves_to_the_board(host):
    addrs = {a[4][0] for a in socket.getaddrinfo("tinyg-switchbox.local", 80, socket.AF_INET)}
    assert host in addrs


def test_rest_info_and_state_match_serial(host, switchbox, record):
    status, info = http_json("GET", f"http://{host}/api/info")
    assert status == 200 and info["boot_id"] == redact(switchbox.info())["boot_id"]
    assert info["wifi"]["mode"] == "sta" and info["wifi"]["ip"] == host
    status, state = http_json("GET", f"http://{host}/api/state")
    assert status == 200 and [c["name"] for c in state["channels"]] == [c["name"] for c in switchbox.state()["channels"]]
    record("http_info", {k: info[k] for k in ("fw_version", "api_version", "heap_free", "uptime_ms")})


def test_rest_commands_reach_the_same_executor(host, switchbox):
    since = switchbox.head()
    status, r = http_json("PUT", f"http://{host}/api/channels/xmin", {"action": "trip", "ttl_ms": 3000})
    assert status == 200 and r["channel"]["state"] == "closed" and r["channel"]["level"] == 0 and r["ttl_ms"] == 3000
    assert switchbox.channel("xmin")["state"] == "closed"            # visible over serial at once
    status, r = http_json("POST", f"http://{host}/api/channels/xmin/pulse", {"action": "release", "us": 300})
    assert status == 200 and r["duration_us"] == 300
    time.sleep(0.05)
    status, r = http_json("PUT", f"http://{host}/api/channels/xmin", {"state": "open"})
    assert status == 200 and r["changed"] is True
    status, page = http_json("GET", f"http://{host}/api/events?since={since}&limit=32")
    edges = [e for e in page["events"] if e["event"] == "edge" and e["ch"] == "xmin"]
    assert [e["state"] for e in edges] == ["closed", "open", "closed", "open"]
    assert {e["src"] for e in edges} == {"http"}
    gap = edges[2]["t_us"] - edges[1]["t_us"]
    assert 300 <= gap <= 330, gap


def test_rest_errors_are_structured(host):
    for method, path, body, code, err in [
            ("PUT", "/api/channels/wmin", {"state": "closed"}, 404, "unknown_channel"),
            ("PUT", "/api/channels/xmin", {"state": "shut"}, 400, "bad_param"),
            ("GET", "/api/state?verbose=1", None, 400, "unknown_param"),
            ("PUT", "/api/channels/xmin", b"{not json", 400, "bad_json"),
            ("POST", "/api/channels/xmin/pulse", {"ms": 5, "action": "release"}, 409, "no_change"),
            ("GET", "/api/nope", None, 404, "not_found")]:
        status, r = http_json(method, f"http://{host}{path}", body)
        assert (status, r["error"]["code"]) == (code, err), (path, status, r)


def test_openapi_json_is_the_checkout_spec(host):
    status, spec = http_json("GET", f"http://{host}/openapi.json")
    assert status == 200 and spec == load_spec()


def test_ui_page_is_self_contained(host):
    status, ctype, raw = http("GET", f"http://{host}/")
    page = raw.decode()
    assert status == 200 and ctype.startswith("text/html")
    assert "<title>TinyG switchbox</title>" in page and "/ws" in page
    assert not re.search(r"""(src|href)\s*=\s*["']?(https?:)?//""", page), "UI must not load external resources"


def test_websocket_snapshot_requests_and_live_events(host, switchbox):
    ws = Ws(host)
    try:
        snap = redact(ws.recv())
        assert snap["event"] == "snapshot" and len(snap["state"]["channels"]) == len(switchbox.config()["channels"])
        ws.send({"id": 41, "op": "setChannel", "name": "ymin", "state": "closed"})
        reply, seen = ws.wait_for(lambda m: m.get("id") == 41)
        assert reply["ok"] and reply["result"]["channel"]["state"] == "closed"
        ws.wait_for(lambda m: m.get("event") == "edge" and m.get("ch") == "ymin" and m["src"] == "ws", timeout=3)
        switchbox.contact("ymin", state="open")                     # a serial command, pushed live to WS
        ws.wait_for(lambda m: m.get("event") == "edge" and m.get("ch") == "ymin" and m["src"] == "serial"
                    and m["state"] == "open", timeout=3)
        ws.send({"id": 42, "op": "getSpec"})
        reply, _ = ws.wait_for(lambda m: m.get("id") == 42)
        assert reply["error"]["code"] == "unsupported_transport"
    finally:
        ws.close()
