"""Local HIL dashboard: audit findings, their tests and branches, run results, and path plots.

  cd tests/hil && uv run python -m tinyg_hil.dashboard [--port 8765] [--runs DIR ...]

Reads (nothing is written):
  findings   git show docs/edge-audit:docs/audit/edge-0.97/findings.json   (or --findings FILE)
  coverage   git show docs/edge-audit:docs/audit/edge-0.97/test-coverage.json (or --coverage FILE)
  branches   refs/heads/fix/E* in the repository
  runs       run records written by the harness (.runs/ next to conftest.py, plus any --runs dirs)
"""
import argparse
import html
import json
import pathlib
import re
import subprocess
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from . import paths, render

HIL = pathlib.Path(__file__).resolve().parent.parent
UID = re.compile(r"test_(e\d{3})_", re.I)
SEVERITY = ["critical", "high", "medium", "low"]
OUTCOME_ORDER = ["failed", "error", "xpassed", "xfailed", "passed", "skipped"]


def git(repo, *args):
    r = subprocess.run(["git", "-C", str(repo), *args], capture_output=True, text=True)
    return r.stdout if r.returncode == 0 else ""


class Data:
    def __init__(self, repo, findings_file, coverage_file, run_dirs):
        self.repo, self.findings_file, self.coverage_file, self.run_dirs = repo, findings_file, coverage_file, run_dirs

    def _json(self, file, ref_path):
        text = pathlib.Path(file).read_text() if file else git(self.repo, "show", ref_path)
        return json.loads(text) if text.strip() else None

    def findings(self):
        data = self._json(self.findings_file, "docs/edge-audit:docs/audit/edge-0.97/findings.json") or []
        return {f["uid"]: f for f in data}

    def coverage(self):
        return self._json(self.coverage_file, "docs/edge-audit:docs/audit/edge-0.97/test-coverage.json") or {}

    def branches(self):
        out = {}
        refs = git(self.repo, "for-each-ref", "--format=%(refname:short) %(objectname:short) %(committerdate:iso8601)",
                   "refs/heads/fix/")
        for line in refs.splitlines():
            name, sha, date = line.split(" ", 2)
            m = re.match(r"fix/(E\d{3})-", name)
            if not m:
                continue
            log = git(self.repo, "log", "--format=%h %s", f"test/hil-harness..{name}").splitlines()
            fw = git(self.repo, "diff", "--name-only", f"test/hil-harness...{name}", "--", "firmware/").split()
            out[m.group(1)] = {"name": name, "sha": sha, "date": date[:16], "commits": log, "firmware_files": fw}
        return out

    def test_source(self, branch, uid):
        files = git(self.repo, "ls-tree", "--name-only", f"{branch}:tests/hil/tests/bugs").split()
        name = next((f for f in files if f.lower().startswith(f"test_{uid.lower()}_")), None)
        return (name, git(self.repo, "show", f"{branch}:tests/hil/tests/bugs/{name}")) if name else (None, "")

    def runs(self):
        out = []
        for d in self.run_dirs:
            for run in sorted(pathlib.Path(d).glob("*/run.json"), reverse=True):
                meta = json.loads(run.read_text())
                if meta.get("identity"):            # hardware runs only
                    meta["dir"] = run.parent
                    out.append(meta)
        return out

    def records_by_uid(self):
        by = {}
        for meta in self.runs():
            for nodeid, t in meta["tests"].items():
                uids = set(t.get("bugs") or [])
                m = UID.search(nodeid)
                if m:
                    uids.add(m.group(1).upper())
                for uid in uids:
                    by.setdefault(uid, []).append((meta, nodeid, t))
        return by


# ---- rendering -----------------------------------------------------------------------------

CSS = """
:root{--bg:#f6f7f9;--panel:#fff;--ink:#1d2330;--muted:#5f6b7a;--line:#d9dee5;--accent:#2457d6;
--ok:#1f7a3f;--bad:#b42318;--warn:#a15c00;--xf:#6941c6;--skip:#667085}
@media (prefers-color-scheme:dark){:root:not([data-theme=light]){--bg:#11151c;--panel:#181e27;--ink:#e6e9ef;
--muted:#9aa4b2;--line:#2b3442;--accent:#7aa2ff;--ok:#4ade80;--bad:#f87171;--warn:#fbbf24;--xf:#c4b5fd;--skip:#98a2b3}}
:root[data-theme=dark]{--bg:#11151c;--panel:#181e27;--ink:#e6e9ef;--muted:#9aa4b2;--line:#2b3442;--accent:#7aa2ff;
--ok:#4ade80;--bad:#f87171;--warn:#fbbf24;--xf:#c4b5fd;--skip:#98a2b3}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.45 system-ui,sans-serif}
header{display:flex;gap:18px;align-items:center;padding:12px 20px;border-bottom:1px solid var(--line);background:var(--panel)}
header b{font-size:15px}header a{color:var(--muted);text-decoration:none}header a:hover{color:var(--accent)}
main{padding:18px 20px;max-width:1400px;margin:0 auto}a{color:var(--accent)}
.cards{display:flex;flex-wrap:wrap;gap:10px;margin-bottom:16px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:10px 14px;min-width:130px;cursor:pointer}
.card .n{font-size:22px;font-weight:650}.card .l{color:var(--muted);font-size:12px}.card.on{outline:2px solid var(--accent)}
table{width:100%;border-collapse:collapse;background:var(--panel);border:1px solid var(--line)}
th,td{text-align:left;padding:6px 9px;border-bottom:1px solid var(--line);vertical-align:top}
th{font-size:12px;color:var(--muted);font-weight:600;position:sticky;top:0;background:var(--panel)}
.pill{display:inline-block;padding:1px 8px;border-radius:10px;font-size:12px;font-weight:600;border:1px solid currentColor}
.passed{color:var(--ok)}.failed,.error,.xpassed{color:var(--bad)}.xfailed{color:var(--xf)}.skipped,.none{color:var(--skip)}
.untestable{color:var(--warn)}.critical{color:var(--bad);font-weight:700}.high{color:var(--warn);font-weight:600}
.muted{color:var(--muted)}pre{background:var(--panel);border:1px solid var(--line);padding:10px;overflow:auto;
font:12px/1.4 ui-monospace,monospace;max-height:520px}section{margin:18px 0}h1{font-size:20px;margin:4px 0 2px}
h2{font-size:15px;margin:20px 0 8px}details{margin:6px 0}summary{cursor:pointer;color:var(--muted)}
.wrap{overflow-x:auto}.plot svg{max-width:100%;height:auto}input[type=search]{padding:6px 9px;border:1px solid var(--line);
border-radius:6px;background:var(--panel);color:var(--ink);width:280px;margin-bottom:10px}
"""

THEME_JS = """<script>
function setTheme(t){try{localStorage.setItem('theme',t)}catch(e){}document.documentElement.dataset.theme=t}
try{const t=localStorage.getItem('theme');if(t)document.documentElement.dataset.theme=t}catch(e){}
</script>"""


def esc(s):
    return html.escape(str(s if s is not None else ""))


def page(title, body):
    return (f"<!doctype html><html><head><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
            f"<title>{esc(title)}</title><style>{CSS}</style>{THEME_JS}</head><body>"
            "<header><b>TinyG HIL</b><a href='/'>Findings</a><a href='/runs'>Runs</a><a href='/branches'>Branches</a>"
            "<span style='flex:1'></span><a href='#' onclick=\"setTheme('light');return false\">light</a>"
            "<a href='#' onclick=\"setTheme('dark');return false\">dark</a></header>"
            f"<main>{body}</main></body></html>")


def pill(outcome):
    return f"<span class='pill {esc(outcome)}'>{esc(outcome)}</span>"


def latest_outcome(records):
    """Most recent run's outcome; within that run the worst outcome across the uid's tests."""
    if not records:
        return None, None
    newest = max(r[0]["started"] for r in records)
    outs = [r[2]["outcome"] for r in records if r[0]["started"] == newest]
    return min(outs, key=lambda o: OUTCOME_ORDER.index(o) if o in OUTCOME_ORDER else 99), newest


def status_of(uid, cov, branches, records):
    if uid in branches:
        out, _ = latest_outcome(records)
        if branches[uid]["firmware_files"]:
            return "fix in progress" if out != "passed" else "fixed"
        return "test written"
    c = cov.get(uid, {})
    return {"untestable": "untestable", "blocked": "blocked", "not-needed": "no test needed",
            "staged": "test staged"}.get(c.get("status"), "no test yet")


def index(data):
    findings, cov, branches, by = data.findings(), data.coverage(), data.branches(), data.records_by_uid()
    rows, counts = [], {}
    for uid in sorted(findings):
        f = findings[uid]
        sev = f["check"].get("corrected_severity") or f["severity"]
        st = status_of(uid, cov, branches, by.get(uid, []))
        counts[st] = counts.get(st, 0) + 1
        out, when = latest_outcome(by.get(uid, []))
        br = branches.get(uid, {}).get("name", "")
        reason = cov.get(uid, {}).get("reason", "") if st in ("untestable", "blocked", "no test needed", "test staged") else ""
        rows.append(f"<tr data-status='{esc(st)}'><td><a href='/f/{uid}'>{uid}</a></td><td class='{esc(sev)}'>{esc(sev)}</td>"
                    f"<td>{esc(f['title'])}{'<div class=muted>' + esc(reason[:180]) + '</div>' if reason else ''}</td>"
                    f"<td>{esc(f['category'])}</td><td>{esc(st)}</td><td class=muted>{esc(br)}</td>"
                    f"<td>{pill(out) if out else '<span class=muted>—</span>'}</td><td class=muted>{esc(when or '')}</td></tr>")
    cards = "".join(f"<div class=card data-filter='{esc(k)}'><div class=n>{v}</div><div class=l>{esc(k)}</div></div>"
                    for k, v in sorted(counts.items(), key=lambda kv: -kv[1]))
    body = (f"<h1>edge-0.97 findings</h1><p class=muted>{len(findings)} findings · latest hardware outcome per finding. "
            "xfailed = the bug reproduced as its test predicts; passed on a branch with firmware changes = fixed.</p>"
            f"<div class=cards><div class='card on' data-filter=''><div class=n>{len(findings)}</div><div class=l>all</div></div>{cards}</div>"
            "<input type=search id=q placeholder='filter by text…'>"
            "<div class=wrap><table id=t><thead><tr><th>ID</th><th>Severity</th><th>Finding</th><th>Category</th><th>Status</th>"
            f"<th>Branch</th><th>Latest</th><th>Run</th></tr></thead><tbody>{''.join(rows)}</tbody></table></div>"
            """<script>
let st='';const q=document.getElementById('q');
function apply(){const s=q.value.toLowerCase();for(const r of document.querySelectorAll('#t tbody tr')){
r.style.display=((!st||r.dataset.status===st)&&(!s||r.textContent.toLowerCase().includes(s)))?'':'none'}}
for(const c of document.querySelectorAll('.card')){c.onclick=()=>{st=c.dataset.filter;
document.querySelectorAll('.card').forEach(x=>x.classList.toggle('on',x===c));apply()}}
q.oninput=apply;
</script>""")
    return page("TinyG HIL findings", body)


def finding(data, uid):
    findings, cov, branches = data.findings(), data.coverage(), data.branches()
    f = findings.get(uid)
    if not f:
        return None
    c, br = cov.get(uid, {}), branches.get(uid)
    parts = [f"<h1>{uid}: {esc(f['title'])}</h1>",
             f"<p class=muted>{esc(f['category'])} · severity {esc(f['check'].get('corrected_severity') or f['severity'])} · "
             f"verdict {esc(f['check'].get('verdict'))} · source {esc(f.get('source_ref'))}</p>"]
    if c:
        parts.append(f"<p><b>Test coverage:</b> {esc(c.get('status'))} — {esc(c.get('reason'))}</p>")
    for label, key in (("Evidence", "evidence"), ("Proposed change", "proposed_change"), ("Hardware test", "hardware_test")):
        if f.get(key):
            parts.append(f"<h2>{label}</h2><p>{esc(f[key])}</p>")
    if br:
        name, src = data.test_source(br["name"], uid)
        commits = "".join(f"<li><code>{esc(x)}</code></li>" for x in br["commits"])
        parts.append(f"<h2>Branch <code>{esc(br['name'])}</code></h2><ul>{commits}</ul>"
                     f"<p class=muted>firmware files changed vs harness: {esc(', '.join(br['firmware_files']) or 'none (test only)')}</p>")
        if src:
            parts.append(f"<details><summary>{esc(name)}</summary><pre>{esc(src)}</pre></details>")
    records = sorted(data.records_by_uid().get(uid, []), key=lambda r: r[0]["started"], reverse=True)
    parts.append(f"<h2>Runs ({len(records)})</h2>")
    for meta, nodeid, t in records[:12]:
        rec = json.loads((meta["dir"] / t["file"]).read_text())
        parts.append(f"<section><div>{pill(rec['outcome'])} <code>{esc(nodeid.split('::', 1)[-1])}</code> "
                     f"<span class=muted>run {esc(meta['started'])} · {esc(meta['branch'])} @ {esc(meta['commit'])} · "
                     f"fb {esc((meta.get('identity') or {}).get('fb'))}</span></div>")
        analysis = paths.analyze(rec)
        if analysis["actual"] or analysis["expected"]:
            parts.append(f"<div class=plot>{render.svg(analysis)}</div>")
        failure = next((p["longrepr"] for p in rec["phases"].values() if p.get("longrepr")), "")
        if failure:
            parts.append(f"<details><summary>failure</summary><pre>{esc(failure)}</pre></details>")
        traffic = sorted([(s["t"], ">>", s["data"]) for s in rec.get("sent", [])] +
                         [(e["t"], "<<", e["raw"]) for e in rec.get("events", [])])
        if traffic:
            text = "\n".join(f"{t:8.3f} {d} {line.rstrip()}" for t, d, line in traffic)
            parts.append(f"<details><summary>serial traffic ({len(traffic)} lines)</summary><pre>{esc(text)}</pre></details>")
        parts.append("</section>")
    return page(f"{uid} · TinyG HIL", "".join(parts))


def runs_page(data):
    rows = []
    for meta in data.runs():
        outs = {}
        for t in meta["tests"].values():
            outs[t["outcome"]] = outs.get(t["outcome"], 0) + 1
        summary = " ".join(f"{pill(k)} {v}" for k, v in sorted(outs.items()))
        rows.append(f"<tr><td><a href='/run/{esc(meta['dir'].name)}'>{esc(meta['started'])}</a></td><td>{esc(meta['branch'])}</td>"
                    f"<td><code>{esc(meta['commit'])}</code></td><td>{esc((meta.get('identity') or {}).get('fb'))}</td>"
                    f"<td>{summary}</td></tr>")
    return page("Runs · TinyG HIL", "<h1>Hardware runs</h1><div class=wrap><table><thead><tr><th>Started (UTC)</th><th>Branch</th>"
                f"<th>Commit</th><th>fb</th><th>Outcomes</th></tr></thead><tbody>{''.join(rows)}</tbody></table></div>")


def run_page(data, name):
    meta = next((m for m in data.runs() if m["dir"].name == name), None)
    if not meta:
        return None
    rows = []
    for nodeid, t in sorted(meta["tests"].items()):
        m = UID.search(nodeid)
        link = f"<a href='/f/{m.group(1).upper()}'>{m.group(1).upper()}</a>" if m else ""
        rows.append(f"<tr><td>{pill(t['outcome'])}</td><td>{link}</td><td><code>{esc(nodeid)}</code></td></tr>")
    return page(f"Run {name}", f"<h1>Run {esc(meta['started'])}</h1><p class=muted>{esc(meta['branch'])} @ {esc(meta['commit'])} · "
                f"{esc(json.dumps(meta.get('identity')))}</p><table><tbody>{''.join(rows)}</tbody></table>")


def branches_page(data):
    findings, by = data.findings(), data.records_by_uid()
    rows = []
    for uid, b in sorted(data.branches().items()):
        out, when = latest_outcome(by.get(uid, []))
        rows.append(f"<tr><td><a href='/f/{uid}'>{uid}</a></td><td><code>{esc(b['name'])}</code></td><td>{len(b['commits'])}</td>"
                    f"<td>{esc(', '.join(b['firmware_files']) or '—')}</td><td>{pill(out) if out else '—'}</td>"
                    f"<td class=muted>{esc(findings.get(uid, {}).get('title', ''))[:120]}</td></tr>")
    return page("Branches · TinyG HIL", "<h1>Bug branches</h1><p class=muted>One branch per finding, on top of test/hil-harness. "
                "Firmware files appear once a fix is committed.</p><div class=wrap><table><thead><tr><th>ID</th><th>Branch</th>"
                "<th>Commits</th><th>Firmware changed</th><th>Latest</th><th>Finding</th></tr></thead>"
                f"<tbody>{''.join(rows)}</tbody></table></div>")


def make_handler(data):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = urllib.parse.urlparse(self.path).path
            try:
                if path == "/":
                    body = index(data)
                elif path == "/runs":
                    body = runs_page(data)
                elif path == "/branches":
                    body = branches_page(data)
                elif path.startswith("/f/"):
                    body = finding(data, path[3:].upper())
                elif path.startswith("/run/"):
                    body = run_page(data, path[5:])
                else:
                    body = None
            except Exception as e:  # noqa: BLE001 - show the error instead of dropping the connection
                body = page("error", f"<pre>{esc(repr(e))}</pre>")
            code = 200 if body else 404
            payload = (body or page("not found", "<h1>Not found</h1>")).encode()
            self.send_response(code)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def log_message(self, *args):
            pass
    return Handler


def main(argv=None):
    p = argparse.ArgumentParser(prog="tinyg_hil.dashboard")
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--repo", default=str(HIL.parent.parent))
    p.add_argument("--findings")
    p.add_argument("--coverage")
    p.add_argument("--runs", action="append", default=[], help="extra run directories (default: tests/hil/.runs)")
    args = p.parse_args(argv)
    data = Data(pathlib.Path(args.repo), args.findings, args.coverage, [str(HIL / ".runs"), *args.runs])
    server = ThreadingHTTPServer((args.host, args.port), make_handler(data))
    print(f"TinyG HIL dashboard on http://{args.host}:{args.port}/")
    server.serve_forever()


if __name__ == "__main__":
    main()
