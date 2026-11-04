"""Overnight sweep: invalid values for every persisted setting, one write at a time with the reply awaited.

For each setting (identity, communication and status-report slot rows excluded) the sweep sends a string, an
array, a boolean, a huge number, a fraction, and a large negative number, and reads the value back after
each. Definite bug: a write refused with status != 0 that still changed the value, or a board that stops
answering. Recorded for review: invalid types accepted (status 0), negatives accepted. Any value that did
change is put back at once (the session also restores every setting at its end).
"""
import json
import math

import pytest

from tinyg_hil import snapshot

pytestmark = [pytest.mark.diagnostic, pytest.mark.persists, pytest.mark.dialect("d449"), pytest.mark.timeout(3600)]

SKIP = set(snapshot.NEVER_WRITE) | set(snapshot.COMM_ORDER) | {"sv", "si", "qv", "tv", "net", "fd", "ma", "sr", "id"}
PROBES = ['"abc"', "[1]", "true", "1e30", "-1e9", "0.5", "1.5", "-1"]


def _same(a, b):
    if isinstance(a, float) or isinstance(b, float):
        try:
            return math.isclose(float(a), float(b), rel_tol=1e-5, abs_tol=1e-6)
        except (TypeError, ValueError):
            return a == b
    return a == b


def test_settings_sweep(tg, record):
    snap = snapshot.take(tg)
    tokens = [t for t in sorted(snap["values"]) if t not in SKIP and not snapshot.INTERNAL.match(t)]
    bugs, review, n = [], [], 0
    for tok in tokens:
        original = snap["values"][tok]
        for probe in PROBES:
            resp = tg.send_line('{"%s":%s}' % (tok, probe), timeout=5.0)
            n += 1
            now = tg.get(tok)
            changed = not _same(now, original)
            r = {"token": tok, "probe": probe, "status": resp.status, "was": original, "now": now}
            if resp.status != 0 and changed:
                bugs.append(r)
            elif resp.status == 0 and probe in ('"abc"', "[1]", "1e30"):
                review.append(r)
            elif resp.status == 0 and probe in ("-1e9", "-1") and changed:
                review.append(r)
            if changed:
                tg.set(tok, original, allow="put back after the sweep probe")
        assert tg.stat() is not None
    record("sweep", {"tokens": len(tokens), "writes": n, "bugs": bugs, "review": review[:200]})
    assert not bugs, json.dumps(bugs[:20])
