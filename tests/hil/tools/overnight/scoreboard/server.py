import os
#!/usr/bin/env python3
"""Arcade high-score board for the overnight bug hunt. Stdlib only: HTTP + a minimal WebSocket push.

usage: server.py <night_dir> [host] [port]
Serves index.html at / and pushes the scoreboard over /ws whenever bugs.json or the bench logs change.
Entries: curated bugs (bugs.json) plus automatic findings (FAILED lines in logs/b1.log and logs/b2.log).
"""
import base64, hashlib, json, os, re, socket, struct, sys, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

NIGHT = sys.argv[1]
HOST = sys.argv[2] if len(sys.argv) > 2 else "0.0.0.0"
PORT = int(sys.argv[3]) if len(sys.argv) > 3 else 9090
HERE = os.path.dirname(os.path.abspath(__file__))
SCORE = {"critical": 99990, "high": 50000, "medium": 10000, "low": 2500, "info": 500}
clients, lock = [], threading.Lock()


RUNS = os.environ.get("TINYG_RUNS", "runs")
try:                                             # run records before this UTC stamp are an earlier hunt
    SINCE = open(os.path.join(NIGHT, "since")).read().strip()
except OSError:
    SINCE = "20261103T0515"
_cache = {}
KINDS = [  # (pattern in the failure text, name, severity, plain title)
    (r"saleae start_capture|INTERNAL_ERROR", "BENCH FAULT", "info", "Instrument, not firmware: the Saleae capture could not start"),
    (r"BlockingIOError|BoardBusy", "BENCH BUSY", "info", "Harness, not firmware: two test sessions wanted the same board (a loop restart)"),
    (r"ALARM during|stat 2,|\(stat 2\)", "ALARM", "high", "The board went into ALARM during a job (see what happened)"),
    (r"StreamLineTooLong", "LONG LINE", "info", "Harness limit, not a firmware bug: a line longer than the 200-byte character-count budget"),
    (r"StreamStalled", "SILENT STALL", "critical", "The board went silent mid-stream: lines unanswered and no status report for 5 s"),
    (r"net steps", "LOST STEPS", "critical", "Step pulses did not add up: after returning to the start a motor's net step count was not zero"),
    (r"not back at the start", "WRONG SPOT", "critical", "After returning to the start the board reported a different position"),
    (r"panicked|PANIC|SHUTDOWN", "PANIC", "critical", "The board panicked or shut down"),
    (r"status 13 \(BUFFER_FULL\)", "BUFFER FULL", "high", "A command was refused as 'buffer full' although the host waited for each reply"),
    (r"seed \d+:", "OFFSET DRIFT", "critical", "Work or machine position disagreed with the coordinate-system / G92 / tool-offset model"),
    (r"must_refuse", "GARBAGE IN", "high", "A malformed G-code line was accepted (status 0) or a refused line moved the machine"),
    (r'"token"', "SETTING SLIP", "high", "A settings write was refused but the value changed anyway"),
    (r"fs q not consecutive", "HARNESS", "info", "Harness mistake, not firmware: the soak's push window did not restart (fixed in the test)"),
    (r"malformed lines", "PUSH GLITCH", "high", "Feed-limit push lines were malformed during a stream"),
    (r"Timeout|timed out", "TIME OUT", "high", "The test timed out waiting for the board"),
    (r"NoResponse", "NO ANSWER", "high", "The board stopped answering or never became idle"),
]


def _detail(t):
    longrepr = " ".join(((t.get("phases") or {}).get(ph) or {}).get("longrepr", "") for ph in ("setup", "call", "teardown"))
    elines = [l.strip()[1:].strip() for l in longrepr.splitlines() if l.strip().startswith("E ")]
    return longrepr, (elines[0] if elines else longrepr.strip().splitlines()[-1] if longrepr.strip() else "")


WT = os.environ.get("TINYG_SCRATCH", "/tmp/tinyg-scratch") + "/wt-night/tests/hil"


def _purpose(node):
    """The test file's own one-line description (its docstring's first line)."""
    try:
        src = open(os.path.join(WT, node.split("::")[0])).read()
        doc = src.split('"""')[1].strip().splitlines()[0]
        return doc.rstrip(".")
    except Exception:
        return node.split("::")[0].split("/")[-1]


def _covered(entry, curated):
    for c in curated:
        for pat in c.get("covers", []):
            if re.search(pat, entry["title"] + " " + entry.get("detail", "") + " " + entry.get("repro", "")):
                return c["id"]
    return None


HISTORY = os.path.join(NIGHT, "history.json")
RESOLVED = ("DUPLICATE", "HARNESS", "VOID", "FIXED", "CAN'T REPRODUCE", "NOT A BUG")


def _ledger(current):
    """Every finding ever seen stays; only its status changes. Keyed by kind + test + file."""
    try:
        hist = json.load(open(HISTORY))
    except Exception:
        hist = {}
    changed = False
    for e in current:
        k = e["key"]
        if k not in hist:
            e["first_seen"] = time.strftime("%H:%M")
            hist[k] = e
            changed = True
        else:
            seen = hist[k].get("first_seen")
            if {**hist[k], "first_seen": None} != {**e, "first_seen": None}:
                hist[k] = {**e, "first_seen": seen}
                changed = True
    if changed:
        json.dump(hist, open(HISTORY, "w"), indent=1)
    return list(hist.values())


def _replications():
    """(test path, file name) -> latest rerun verdict from the loops' replications.jsonl."""
    out = {}
    try:
        for line in open(os.path.join(NIGHT, "replications.jsonl")):
            j = json.loads(line)
            m = re.search(r"SOAK_FILE=(\S+)", j.get("env", ""))
            out[(j["test"], os.path.basename(m.group(1)) if m else "")] = j
    except Exception:
        pass
    return out


def auto_findings():
    out, seen = [], set()
    reps = _replications()
    try:
        dirs = sorted(d for d in os.listdir(RUNS) if d >= SINCE)
    except OSError:
        return out
    for d in dirs:
        for f in sorted(os.listdir(os.path.join(RUNS, d, "tests")) if os.path.isdir(os.path.join(RUNS, d, "tests")) else []):
            path = os.path.join(RUNS, d, "tests", f)
            try:
                mt = os.path.getmtime(path)
                if path not in _cache or _cache[path][0] != mt:
                    _cache[path] = (mt, json.load(open(path)))
                t = _cache[path][1]
            except Exception:
                continue
            if t.get("outcome") not in ("failed", "error"):
                continue
            longrepr, msg = _detail(t)
            raw = t.get("observations") or {}
            obs = raw if isinstance(raw, dict) else {o.get("name"): o.get("value") for o in raw if isinstance(o, dict)}
            soak = obs.get("soak_file") or {}
            name, sev, title = "TEST FAILED", "high", None
            for text in (msg, longrepr):                 # the error line decides; the full text only if it does not
                hit = next(((n, s, ttl) for pat, n, s, ttl in KINDS if re.search(pat, text)), None)
                if hit:
                    name, sev, title = hit
                    break
            phases = t.get("phases") or {}
            if (phases.get("setup") or {}).get("longrepr") and not (phases.get("call") or {}).get("longrepr"):
                name, sev, title = "BENCH SETUP", "info", "Bench, not firmware: the test could not start (the board was left in a bad state)"
            if title is None:
                title = "Failed: " + _purpose(t.get("nodeid", ""))
            node = t.get("nodeid", "?")
            src = os.path.basename(soak.get("file", "")) if soak else ""
            key = (name, node.split("[")[0], src)
            if key in seen:
                continue
            seen.add(key)
            keystr = "|".join(key)
            env = []
            if soak:
                env += [f"SOAK_FILE={soak.get('file')}", f"SOAK_MODE={soak.get('mode')}"]
            rep = reps.get((node.split("::")[0], src))
            edge = {"new": "new in edge-next", "old": "also in edge-0.97", "unclear": "edge-0.97 inconclusive"}.get(
                (rep or {}).get("edge097", ""), "untested")
            if rep and rep.get("verdict") == "bench-error":
                state, edge, sev = "VOID: the rerun could not start (bench setup error)", "untested", "info"
            elif rep and rep.get("void"):
                state, edge, sev = "VOID: " + rep["void"], "untested", "info"
            elif sev == "info":
                state = "HARNESS: not a firmware bug"
            elif rep and rep["verdict"] == "replicated":
                state = "OPEN: REPLICATED after a board reset (same inputs)"
            elif rep:
                state = f"CAN'T REPRODUCE: passed again after a reset (job before it: {rep.get('previous') or '?'})"
            else:
                state = "OPEN: automatic finding, rerun pending"
            out.append({"key": keystr, "initials": name, "severity": sev, "auto": True, "state": state,
                        "title": title + (f" ({src}, {soak.get('mode')} mode)" if src else f" ({node.split('::')[0].split('/')[-1]})"),
                        "detail": msg[:220], "repro": " ".join(env + ["pytest", node.split("[")[0]]) + f"  [run {d}]",
                        "edge_proper": edge, "status": state,
                        "when": d[9:13]})
    return out


def locked_boards():
    """BOARD LOCKED lines the loops write before an ICE reset: a dead main loop is a finding in its own right."""
    out = []
    for bench in ("b1", "b2"):
        path = os.path.join(NIGHT, "logs", f"{bench}.log")
        if not os.path.exists(path):
            continue
        for line in open(path, errors="replace"):
            if "BOARD LOCKED" not in line:
                continue
            when, rest = line[:14], line[line.index("BOARD LOCKED"):].strip()
            pc = re.search(r"pc: (.*?) \|", rest)
            out.append({"key": f"LOCKED|{bench}|{when}", "initials": "BOARD LOCKED", "severity": "critical", "auto": True,
                        "title": f"The board stopped answering and ignored ^X (main loop dead); CPU at {pc.group(1).strip() if pc else '?'} ({bench})",
                        "detail": rest[:300], "repro": "see the job named in the detail; the loop reset the board through the ICE and went on",
                        "edge_proper": "also in edge-0.97" if "wt-edge097" in rest else "untested",
                        "status": "OPEN: caught live; snapshot in the log line", "state": "OPEN", "when": when[6:11].replace(":", "")})
    return out


def board():
    try:
        curated = json.load(open(os.path.join(NIGHT, "bugs.json")))
    except Exception:
        curated = []
    autos = auto_findings() + locked_boards()
    for e in autos:
        dup = _covered(e, curated)
        if dup:
            e["status"] = f"DUPLICATE of {dup}"
    entries = curated + _ledger(autos)
    for i, e in enumerate(entries):
        e.setdefault("id", f"A{i:02d}")
        e["resolved"] = any(e.get("status", "").upper().startswith(s) for s in RESOLVED)
        e["score"] = SCORE.get(e.get("severity", "info"), 500) + (5 if e.get("edge_proper") == "new in edge-next" else 0)
    entries.sort(key=lambda e: (-e["score"], e["resolved"]))   # high score first, whatever the status
    stats = {}
    for bench in ("b1", "b2"):
        path = os.path.join(NIGHT, "logs", f"{bench}.log")
        lines = open(path, errors="replace").read().splitlines() if os.path.exists(path) else []
        stats[bench] = {"runs": len(lines), "failed": sum(1 for l in lines if " failed" in l or " error" in l),
                        "last": lines[-1][:160] if lines else ""}
    return {"entries": entries, "stats": stats, "time": time.strftime("%H:%M:%S")}


def ws_send(conn, text):
    data = text.encode()
    hdr = bytes([0x81]) + (bytes([len(data)]) if len(data) < 126 else
                           bytes([126]) + struct.pack(">H", len(data)) if len(data) < 65536 else
                           bytes([127]) + struct.pack(">Q", len(data)))
    conn.sendall(hdr + data)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == "/ws" and self.headers.get("Upgrade", "").lower() == "websocket":
            key = self.headers["Sec-WebSocket-Key"]
            accept = base64.b64encode(hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()).decode()
            self.send_response(101)
            self.send_header("Upgrade", "websocket")
            self.send_header("Connection", "Upgrade")
            self.send_header("Sec-WebSocket-Accept", accept)
            self.end_headers()
            conn = self.connection
            ws_send(conn, json.dumps(board()))
            with lock:
                clients.append(conn)
            try:
                while conn.recv(1024):
                    pass
            except OSError:
                pass
            with lock:
                if conn in clients:
                    clients.remove(conn)
            return
        if self.path in ("/", "/index.html"):
            body = open(os.path.join(HERE, "index.html"), "rb").read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == "/board.json":
            body = json.dumps(board()).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404)


def watcher():
    last = None
    while True:
        snap = json.dumps(board())
        sig = hashlib.sha1(snap.encode()).hexdigest()
        if sig != last:
            last = sig
            with lock:
                for c in list(clients):
                    try:
                        ws_send(c, snap)
                    except OSError:
                        clients.remove(c)
        time.sleep(2)


threading.Thread(target=watcher, daemon=True).start()
ThreadingHTTPServer((HOST, PORT), Handler).serve_forever()
