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

**Dashboard.**
- Every pytest run with a board writes a run record to `.runs/<UTC>-<branch>/`. Each record holds the traffic each test sent and received, its outcome, and its bug IDs. Motion tests also record their start position.
- `uv run python -m tinyg_hil.dashboard` serves them at http://127.0.0.1:8765/. It shows:
  - every audit finding with its test status, branch and latest hardware outcome;
  - why a finding has no test (from `docs/audit/edge-0.97/test-coverage.json` on the docs branch);
  - per-finding pages with the evidence, test source, failure text and serial traffic.
- **Path plots.** For motion tests, `tinyg_hil/gsim.py` simulates where the sent G-code should go, following NIST RS274NGC: lines, IJK/R arcs in G17/G18/G19, units, distance modes and offsets. `paths.py` overlays the positions the board actually reported.
- `render.py` draws expected (dashed) against board-reported (solid) paths in XY and XZ. It also reports the largest deviation and the end-point error, so a test can assert on the path and not just the endpoint.

**Sharing the board.** Every pytest session and every `python -m tinyg_hil` command takes an exclusive lock on the port (`/tmp/tinyg-hil-<port>.lock`). Another user waits up to `--lock-wait` seconds (default 600), then fails with a message naming who holds the board and why. Set `TINYG_OWNER` (for example your name) so the other side knows who is on the board. Ad-hoc scripts that open the port directly bypass the lock; use the harness instead.

**Atmel-ICE (peek/poke).** With the ICE on the PDI header (AVR port), `tinyg_hil/ice.py` attaches to the running firmware.
- It reads and writes RAM by symbol, including struct members (`mb.buffers_available`, `cm.hold_state`), reads the program counter as `function+offset`, and dumps EEPROM.
- Each access halts the CPU for a few milliseconds.
- Symbols come from the ELF that `tinyg_hil flash` recorded for the build on the board. The Makefile builds with `-g` for DWARF.

```
uv run python -m tinyg_hil.ice peek mb.buffers_available cm.hold_state
uv run python -m tinyg_hil.ice watch cm.hold_state mb.buffers_available --interval 0.1
uv run python -m tinyg_hil.ice where
uv run pytest --port ... --ice           # enables tests marked @pytest.mark.ice (fixture: ice)
```

**Switchbox (electrical switches).** A Feather HUZZAH ESP8266 wired to the TinyG's limit/homing inputs opens and closes them as a real switch would: open-drain contacts, timed pulses and bounces on the board, ttl auto-release, and an event log with µs timestamps. The box itself (firmware, wiring, API, limits) lives in its own repository, synthetos/tinyg-switchbox. This harness keeps the client (`tinyg_hil/switchbox.py`, with a pinned copy of the API in `tinyg_hil/data/switchbox-openapi.json`), the `switchbox` fixture and the tests that use the box.

```
uv run pytest tests/test_switchbox.py --switchbox --switchbox-port /dev/cu.usbserial-XXXX   # fixture: switchbox
uv run python -m tinyg_hil.switchbox info --port /dev/cu.usbserial-XXXX
```


### Failed flash: AVR109 recovery only

Every application transport open first sends only ESC/S and checks for the
`XBoot++` signature. With no unsafe-flash marker, the probe may itself enter a
healthy bootloader during its reset entry window. In that case open sends AVR109
`E` exactly once, drains the ACK/banner until quiet (bounded to three seconds),
then re-probes with ESC/S. It logs the attempt and result in the transcript and
standard warning log. A remaining signature or an exception closes and
quarantines the port before any JSON, G-code, reset or XON byte is sent. An
existing unsafe-flash marker blocks open before any probe or auto-exit.
A flash creates the same persistent per-port guard before entering the bootloader. Failure leaves it in place;
ordinary open/write calls, including teardown and later sessions, refuse traffic.
The marker is `TINYG_HIL_STATE/tinyg-hil-<port>.flash-unsafe` (under the normal
persistent state directory when the variable is unset). Do not delete it simply
to make a session connect.

Recovery uses `tinyg_hil flash --no-snapshot --restore SAVED.json --hex IMAGE.hex`.
This skips application bootstrap, probes with ESC/S and retries programming
through AVR109 directly. It never sends a boot JSON request to a responding
bootloader. If a quarantined port does not return the signature, stop and use
the ICE; do not send wake/reset text. Only a verified programming run with no
remaining bootloader signature clears the guard before reconnecting and restoring
settings. Physical recovery/ICE verification may require the operator to retire
the guard explicitly after verifying the application; absence of a signature
alone is not sufficient to clear a failed-flash guard.

All avrdude calls retain durable start/output/error records under
`TINYG_HIL_STATE/flash-attempts`. A cleanup failure preserves the first error.
Use the updated harness for subsequent sessions: older worktrees do not enforce
this guard. The new bootloader protections are an additional layer, not a reason
to send application text into AVR109.
