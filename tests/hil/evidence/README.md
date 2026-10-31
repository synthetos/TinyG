# Instrument-backed evidence

Store captures for motion, timing and electrical claims here. Ordinary logical
HIL results remain in the run-record store. The initial promotion helper supports
motion-profile captures; other instrument schemas need their own analysis before
promotion. No threshold verdict is inferred from a plot.

Install the optional `evidence` dependency (matplotlib), then, from `tests/hil`:

```
python -m tinyg_hil evidence promote /path/to/run --finding E015 --label before
python -m tinyg_hil evidence promote /path/to/fixed-run --finding E015 --label after \
  --before evidence/E015/<baseline-commit>-before
```

This command never opens a board. It preserves representative `.sal` bytes, stores their raw step edges as gzip,
compacts derived JSON without duplicating traces, and renders labelled PNGs from
the stored data. `summary.json` records firmware/build identity, harness and
promotion revisions, bench identity, source-run path, method, limitations and
artifact hashes. The default representatives are 0.05, 1 and 10 mm; select others with
`--representative`. Full-sweep raw artifacts remain in the source run, referenced
by path and SHA-256. Target less than 1–1.5 MB per finding; files larger than
1 MB are explicitly reported. Existing evidence
directories are never overwritten.

Motion overlays use the same baseline-selected fit window on both builds. Raw
interval derivatives and smoothed derivatives are shown separately because step
quantization magnifies higher derivatives. Time starts at the first measured step;
this is not command-to-rest time. Limits and estimator limitations must accompany
any claim. Keep unresolved cases unresolved.

When a fix actually passes its gates, its merge message should contain an
`Evidence: tests/hil/evidence/<finding>/<commit>-after/` trailer. A stored capture
or a code-review ACK by itself does not establish that those gates passed.
