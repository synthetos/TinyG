"""Per-test run records for the dashboard: what was sent, what came back, and the outcome.

A run is a directory .runs/<UTC stamp>-<branch>/ holding run.json and tests/<nodeid>.json.
Each test record has the test body's traffic only (after the per-test setup), its bug markers,
outcome (passed/failed/xfailed/xpassed/skipped/error) and, for motion tests, the start position.
"""
import datetime
import json
import pathlib
import re
import subprocess
import time

POSITION_TOKENS = [f"{p}{a}" for p in ("pos", "mpo") for a in "xyz"]


def _git(cwd, *args):
    try:
        return subprocess.run(["git", *args], cwd=cwd, capture_output=True, text=True, check=True).stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return ""


class RunRecorder:
    def __init__(self, root, source_dir):
        stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.branch = _git(source_dir, "symbolic-ref", "--short", "-q", "HEAD") or \
            _git(source_dir, "name-rev", "--name-only", "--exclude=tags/*", "HEAD") or "detached"
        self.commit = _git(source_dir, "rev-parse", "--short", "HEAD")
        safe_branch = re.sub(r"[^\w.-]+", "_", self.branch)
        self.dir = pathlib.Path(root) / f"{stamp}-{safe_branch}"
        (self.dir / "tests").mkdir(parents=True, exist_ok=True)
        self.meta = {"started": stamp, "branch": self.branch, "commit": self.commit, "identity": None,
                     "dialect": None, "tests": {}}
        self.pending = {}           # nodeid -> partial record
        self._write_meta()

    def _write_meta(self):
        (self.dir / "run.json").write_text(json.dumps(self.meta, indent=1) + "\n")

    def board(self, tg, purpose=None):
        self.meta["identity"], self.meta["dialect"] = dict(tg.identity), tg.dialect.name
        self.meta["firmware"] = firmware_identity(tg.transport.port)
        self.meta["purpose"] = purpose or ("stock-baseline" if self.meta["firmware"].get("stock") else "fix-verify")
        self._write_meta()

    def begin(self, nodeid, tg, bugs, markers):
        rec = {"nodeid": nodeid, "bugs": bugs, "markers": markers, "phases": {},
               "history_start": len(tg.transport.history), "sent_start": len(tg.transport.sent),
               "t0": time.monotonic(), "start_position": None}
        if "motion" in markers:
            try:
                rec["start_position"] = {t: tg.get(t) for t in POSITION_TOKENS}
            except Exception:  # noqa: BLE001 - a position read failure must not fail the test
                pass
            rec["history_start"], rec["sent_start"] = len(tg.transport.history), len(tg.transport.sent)
        self.pending[nodeid] = rec

    def end(self, nodeid, tg):
        rec = self.pending.get(nodeid)
        if rec is None or tg is None:
            return
        t0 = rec["t0"]
        rec["sent"] = [{"t": round(t - t0, 4), "data": d.decode(errors="replace")}
                       for t, d in tg.transport.sent[rec.pop("sent_start"):]]
        rec["events"] = [{"t": round(e.t - t0, 4), "kind": e.kind, "raw": e.raw}
                         for e in tg.transport.history[rec.pop("history_start"):]]

    def report(self, report):
        rec = self.pending.setdefault(report.nodeid, {"nodeid": report.nodeid, "bugs": [], "markers": [],
                                                      "phases": {}, "start_position": None})
        outcome = report.outcome
        if hasattr(report, "wasxfail"):
            outcome = "xfailed" if report.skipped else "xpassed"
        elif report.when != "call" and report.failed:
            outcome = "error"
        rec["phases"][report.when] = {"outcome": outcome, "duration": round(report.duration, 3),
                                      "longrepr": str(report.longrepr)[-4000:] if report.failed or
                                      (report.skipped and report.when == "setup") else ""}
        if report.when == "teardown":
            rec["outcome"] = _overall(rec["phases"])
            rec.pop("t0", None)
            name = re.sub(r"[^\w.-]+", "_", report.nodeid)[-180:]
            (self.dir / "tests" / f"{name}.json").write_text(json.dumps(rec, indent=1) + "\n")
            self.meta["tests"][report.nodeid] = {"outcome": rec["outcome"], "bugs": rec["bugs"], "file": f"tests/{name}.json"}
            self._write_meta()
            del self.pending[report.nodeid]

    def finish(self):
        self.meta["finished"] = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self._write_meta()


def firmware_identity(port):
    """What is on the board: the provenance tinyg_hil.flash recorded when it flashed it.

    Captured at flash time (commit, firmware tree hash, dirty flag, ELF/hex SHA-256), so a worktree
    rebased or rebuilt afterwards cannot change it. This is declared provenance, not a readback of the
    binary on the chip. The test checkout's branch says nothing about the firmware.
    """
    from .flash import read_provenance
    prov = read_provenance(port)
    if not prov or prov.get("legacy") or not prov.get("commit"):
        return {"known": False, **({"elf": prov["elf"]} if prov.get("elf") else {})}
    return {"known": True, "commit": prov["commit"][:12], **{k: prov[k] for k in
            ("branch", "tree", "dirty", "stock", "stock_ref", "elf", "elf_sha256", "hex_sha256", "flashed_at") if k in prov}}


def _overall(phases):
    setup, call = phases.get("setup", {}), phases.get("call", {})
    if setup.get("outcome") in ("error", "failed"):
        return "error"
    if setup.get("outcome") == "skipped":
        return "skipped"
    result = call.get("outcome", "error")
    if phases.get("teardown", {}).get("outcome") == "error" and result in ("passed", "xfailed"):
        return "error"
    return result
