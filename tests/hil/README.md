# TinyG hardware-in-the-loop tests

pytest suite that drives a real TinyG over its USB serial port. It speaks both the 440.x (master) and 449.x (edge-0.97) protocols.

```
cd tests/hil
uv run pytest --port /dev/cu.usbserial-XXXX                 # offline self-tests + hardware tests
uv run pytest tests/test_harness_offline.py                 # no board needed
uv run python -m tinyg_hil info --port /dev/cu.usbserial-XXXX
uv run python -m tinyg_hil flash --port ... --build ../..   # build this checkout, flash via xboot, keep settings
```

**Safety**
- The port is never guessed. Pass `--port` or set `TINYG_PORT`.
- Anything that could move axes or switch outputs is refused unless `--allow-motion` is given (`tinyg_hil/safety.py`).
- At session start the harness reads every persisted setting into `.snapshots/`, then switches the response format to `js=1 jv=4 ee=0 ec=0 tv=1 sv=1 qv=0`. At the end it writes back anything that differs.

**Known bugs** are tests marked `@pytest.mark.bug("E0xx", on=["d449"])`, where the ID is from `docs/audit/edge-0.97/findings.json`.
- On the listed dialects the test runs as a strict xfail: it must fail.
- The PR that fixes the bug removes the marker, so the test then has to pass.

**Tables.** `tinyg_hil/data/d440.json` and `d449.json` hold the token manifest and status codes. Regenerate them with `python tools/gen_tables.py <git-ref> <name>` (needs avr-gcc) whenever cfgArray or error codes change.

**Settings across builds.**
- `snapshot`/`restore` go through token names, so a 440 snapshot can be restored onto a 449 build.
- Tokens missing on the target are reported as dropped.
- `se00`-`se29` are never copied: they hold build-specific cfgArray row numbers (E140). The status-report list is restored by name instead.
