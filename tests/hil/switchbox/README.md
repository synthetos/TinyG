# TinyG HIL switchbox

An Adafruit Feather HUZZAH ESP8266 whose GPIOs are wired to a TinyG v8's limit/homing switch inputs, so tests can open and close "switches" electrically, the way a real switch does. The TinyG's own edge detection, ISR, lockout, limit and homing logic run exactly as with a physical switch. Nothing on the TinyG side changes: no test-hooks build and no Atmel-ICE are needed.

| path | what |
|---|---|
| `openapi.json` | the API (OpenAPI 3.1). The single source of truth: the firmware embeds it, the Python client and the tests read it |
| `firmware/` | PlatformIO project (Arduino core for ESP8266) |
| `firmware/lib/sbcore/` | the core: request validation, channel model, edge sequencer, event ring. Plain C++, compiled on the host by `tests/host/test_switchbox_core.py` |
| `firmware/src/main.cpp` | ESP8266 layer: pins, timer, serial, WiFi, HTTP, WebSocket, LittleFS |
| `firmware/ui/index.html` | the web UI served at `/` |
| `../tinyg_hil/switchbox.py` | Python client `SwitchBox` and a CLI |

## Electrical model

TinyG v8 switch inputs are xmega pins at 3.3 V with the internal pull-up enabled and both-edge pin interrupts (`firmware/tinyg/switch.c`). A real switch shorts the input to GND; `$st` says whether TinyG reads it as normally open (0) or normally closed (1).

Each switchbox channel emulates an **open-drain contact** between one TinyG input and GND:

| contact | ESP8266 pin | TinyG line |
|---|---|---|
| `closed` | output, latch low | pulled to GND (reads 0) |
| `open` | input, high-Z, no pull-up | raised by TinyG's pull-up (reads 1) |

- The pin is **never driven high**. Only the output enable changes; the output latch is cleared before every enable. GPIO0-15 also have the pad's open-drain driver set, so a stray latch-high write could not source current either. GPIO16 has no open-drain mode and relies on the latch rule alone.
- At power-on, after any reset or crash, and after `releaseAll`, every pin is an input: **all contacts open, nothing driven.**
- **A common GND wire between the Feather and the TinyG is required.** Without it nothing works, and the readback is meaningless.
- Both sides are 3.3 V logic; no level shifting.
- If a real switch is also on an input, put the channel in parallel with a normally-open switch (wired-OR to GND). Disconnect NC switches; a closed NC switch holds the line low whatever the switchbox does.

### NO/NC semantics

Each channel has a configured type, `no` or `nc`, which should match the bench's `$st`. The API accepts either level:

| | NO channel | NC channel |
|---|---|---|
| `action: "trip"` | contact closed | contact open |
| `action: "release"` | contact open | contact closed |
| `state: "closed"/"open"` | the literal contact | the literal contact |
| `releaseAll` | open (untripped) | open (**tripped**) |
| `idleAll` | open | closed |

`releaseAll` is the electrical safe state: nothing driven. An NC limit then reads as tripped, just as a broken wire would. Use `idleAll` to un-trip every channel.

### Readback

Every reply and event carries the actual line levels read with `digitalRead`-equivalent register reads. A channel's `warning` (a warning, not an error) flags:

- closed but the line reads high: something else drives it high, or the pin or wire is faulty;
- open but the line reads low (more than 1 ms after it was released): a real switch or another device holds it low, a wiring fault, or TinyG is unpowered or disconnected and the input floats;
- open and the line toggles by itself: the input is floating (TinyG unpowered, or the wire is off).

The lines are also polled every 10 ms. A change on an undriven line (or a driven line reading high) becomes a `line` event, which shows TinyG power and wiring in the log. The rise of a line after its contact opens also shows up as a `line` event.

## Wiring and pins

Default mapping (configurable at runtime with `setConfig`, persisted in LittleFS):

| channel | Feather pin | GPIO | TinyG input | notes |
|---|---|---|---|---|
| `xmin` | 4 | 4 | X min | |
| `xmax` | 5 | 5 | X max | |
| `ymin` | 12 | 12 | Y min | |
| `ymax` | 13 | 13 | Y max | |
| `zmin` | 14 | 14 | Z min | |
| `zmax` | 16 | 16 | Z max | GPIO16 has no pin interrupt (irrelevant here). Do not bridge 16 to RST for deep sleep: closing `zmax` would then reset the ESP |
| `amin` | 0 | 0 | A min | **boot-strap pin**, see below |
| `amax` | 2 | 2 | A max | **boot-strap pin**, see below |
| — | GND | — | GND | **required** |

Channel names are free strings (1-15 of `A-Z a-z 0-9 _ - .`); tests use `<axis><min|max>`.

**Pins that are refused** (`setConfig` returns `invalid_config`):
- **GPIO15** must be low when the ESP8266 boots. TinyG's pull-up would hold it high and the ESP would not boot from flash.
- GPIO1 and GPIO3 are the USB serial port.
- GPIO6-11 are wired to the SPI flash.
- The ADC pin (A0) is analog-only.

**GPIO0 and GPIO2 are usable with caveats.** They must be high when the ESP8266 boots. TinyG's pull-up keeps them high, and an open channel is high-Z, so normally they are fine. But:
- a closed contact, or a real switch closed on that input, while the ESP resets stops it booting;
- they carry the Feather's LEDs (GPIO0 red, GPIO2 blue; lit while the contact is closed) and GPIO0 its button. Pressing the button grounds `amin`;
- the CP2104 auto-reset circuit pulls GPIO0 low whenever DTR is asserted without RTS. Flashing the board does exactly that to enter the bootloader, and some serial tools do it on open/close. TinyG then sees `amin` close. Do not flash the switchbox while TinyG is homing or running;
- when TinyG is unpowered, the LED path can back-feed a small current into its input clamp.

Hence the default map uses them last, for the A axis.

## Build and flash

```
cd tests/hil/switchbox/firmware
pio run                                              # build
pio run -t upload --upload-port /dev/cu.usbserial-XXXX
```

- Everything is pinned in `platformio.ini`: platform `espressif8266@4.2.1`, Arduino core `3.30102.0` (3.1.2), `ArduinoJson@7.4.3`, `links2004/WebSockets@2.7.3`.
- The CP2104's DTR/RTS auto-reset puts the ESP8266 into its bootloader for the upload; no buttons needed.
- The spec and the UI are embedded at build time (`scripts/embed_assets.py` writes `src/generated/assets.h`, not committed).
- Find the port from USB descriptors, not by opening ports: the CP2104 is VID `0x10C4`, PID `0xEA60` (`ioreg -p IOUSB -l`, `system_profiler SPUSBDataType`). It appears as `/dev/cu.usbserial-<serial>` or `/dev/cu.SLAB_USBtoUART`. Never the TinyG bench ports.

### Why ESP8266WebServer + arduinoWebSockets, not ESPAsyncWebServer

Both the HTTP server (core's `ESP8266WebServer`) and the WebSocket server (`WebSockets4WebServer` from Links2004/arduinoWebSockets, on the same port 80 at `/ws`) are polled from `loop()`. So serial, HTTP and WebSocket requests all run in the same context, through one executor, serialized with no locks. Async handlers run in the lwIP callback context; they would need a cross-context queue and deferred replies, and AsyncWebSocket's per-client queue is a known heap hazard on the ESP8266's ~40 KB.

The cost is that a slow HTTP client can stall `loop()` briefly. Timed edges do not depend on `loop()`: they run from the timer1 interrupt. WebSocket pushes check each client's TCP buffer first and never block on a slow client.

## Provisioning WiFi

Credentials never go in source or git. Provision over USB serial:

```
cd tests/hil
uv run python -m tinyg_hil.switchbox wifi --port /dev/cu.usbserial-XXXX --ssid "ShopNet"   # prompts for the password
uv run python -m tinyg_hil.switchbox wifi --port /dev/cu.usbserial-XXXX --ssid ""          # forget them
```

- The credentials are stored in LittleFS (`/wifi.json`). They are never echoed, logged or returned by any op.
- The board reconnects about 300 ms after the reply.
- With no credentials, or if the station connection fails within 20 s, the board starts an access point:
  - SSID `tinyg-switchbox-XXXX` (last MAC bytes), password `tinyg-switchbox` (documented default);
  - the board is at `192.168.4.1`.
- mDNS name: `tinyg-switchbox.local`, in both modes.
- `getInfo` reports the WiFi mode, SSID, IP and RSSI.

**Security.** HTTP and the WebSocket have no authentication. Anyone on the same network, or anyone in range of the AP who knows the documented password, can trip the CNC's limit switches. Use station mode on a trusted network, or leave the board on USB only.

## API overview

One executor, three transports, the same operations. The `op` names are the spec's `operationId`s.

| op | REST | does |
|---|---|---|
| `getInfo` | `GET /api/info` | firmware/API version, spec SHA-256, uptime, boot id, heap, WiFi, mDNS |
| `getState` | `GET /api/state` | every channel: name, gpio, type, contact state, tripped, line level, last change, pending pulse/bounce, ttl, warning |
| `setChannel` | `PUT /api/channels/{name}` | `state: closed\|open` or `action: trip\|release`, optional `ttl_ms` |
| `pulseChannel` | `POST /api/channels/{name}/pulse` | `ms` or `us`, `action` (default trip) or `state`: timed on the board |
| `bounceChannel` | `POST /api/channels/{name}/bounce` | `edges`, `interval_us`, `final_state` or `final_action`, optional `ttl_ms` |
| `releaseAll` | `POST /api/release_all` | every contact open, all timers cancelled |
| `idleAll` | `POST /api/idle_all` | every channel untripped for its type |
| `getConfig` / `setConfig` | `GET/PUT /api/config` | mapping and types; validated as a whole, persisted; `{"defaults": true}` |
| `getEvents` | `GET /api/events?since=&limit=` | event history (pages of up to 32) |
| `setWifi` | `PUT /api/wifi` | station credentials |
| `getSpec` | `GET /openapi.json` | the spec; over serial, one line, so the API is learnable without WiFi |
| `subscribe` | (serial only) | event lines on the serial connection |

**Serial** (115200 8N1): newline-delimited JSON.

```
> {"id":1,"op":"setChannel","name":"xmin","action":"trip","ttl_ms":2000}
< {"id":1,"ok":true,"result":{"changed":true,"ttl_ms":2000,"channel":{"name":"xmin","gpio":4,"type":"no","state":"closed",...}}}
> {"id":2,"op":"pulseChannel","name":"ymin","us":200}
> {"id":3,"op":"setChannel","name":"wmin","state":"closed"}
< {"id":3,"ok":false,"error":{"code":"unknown_channel","msg":"unknown channel 'wmin'"}}
```

- After a reset the board prints one `{"event":"boot",...}` line. The ESP8266 ROM first prints a banner at 74880 baud; clients skip lines that are not JSON objects.
- Event lines (`{"event":...}`) only come after `subscribe`.
- Opening the port must not reset the board. Open it with DTR and RTS deasserted (pyserial: create `Serial()` unopened, set `dtr = rts = False`, then `open()`). `SwitchBox` does this and verifies it with `getInfo` (uptime against time since open, plus the boot line).

**REST**: `curl -X PUT -H 'Content-Type: application/json' -d '{"action":"trip","ttl_ms":500}' http://tinyg-switchbox.local/api/channels/xmin`. The response body is the op's result, or `{"error":{"code","msg"}}` with 400/404/409/500.

**WebSocket** `/ws`:
- On connect: a snapshot (info, config, state), then the last 16 events.
- Then every event is pushed live: commands from any source, edges, line changes.
- Clients send requests in the serial framing.
- Pushes are throttled: at most 6 per client per 20 ms, only while the client's TCP buffer has room. A client more than 24 events behind skips ahead and gets `{"event":"dropped","count":N}` plus a fresh state.

**Web UI** at `/` (no external resources, so it works on the isolated AP):
- channel grid: contact, tripped/released, line level, warnings;
- click a channel to toggle it, or use its Pulse button;
- Release all / Idle all;
- a live log of every command and edge with source, request id, µs timestamp and result;
- the mapping editor;
- a connection indicator.

**Events.** Each event has:
- `seq` (monotonic);
- `t_us` (64-bit µs since boot, `micros64()`, so no 71-minute wrap);
- `src`: `serial`, `http`, `ws`, `ttl`, `poll`, `boot` or `sys`;
- `rid`: the request id, when it was a positive integer;
- kind-specific fields;
- `levels` and `driven`: one character per channel, in config order.

What gets logged:
- Every state change.
- Every command that changes something or fails.
- Successful read-only queries (`get*`) are **not** logged, so polling cannot push real history out of the 128-entry ring.
- An edge's `levels` are sampled right at the edge, before a released line has risen. The following `line` event shows the rise.

**ttl.** `setChannel`/`bounceChannel` with `ttl_ms` open the contact by themselves after that long (source `ttl`), even if the test process died. The ttl is a hard limit: it also ends a running pulse or bounce. A later `setChannel` replaces it.

**Errors.** Unknown ops, channels and parameters are rejected with structured errors (`unknown_op`, `unknown_channel`, `unknown_param`, ...). Numeric parameters out of range are clamped and listed in the result's `clamped` array. Negative, fractional and wrongly typed values are rejected.

## Using it from tests

```
uv run pytest tests/test_switchbox.py --switchbox --switchbox-port /dev/cu.usbserial-XXXX \
    --runs-dir .analysis/hil-runs
```

- `--switchbox` un-skips `@pytest.mark.switchbox` tests.
- The port comes from, in order:
  - `--switchbox-port` (or `TINYG_SWITCHBOX_PORT`);
  - the `switchbox` entry of `--port`'s bench in the bench file;
  - for a run without `--port`, the bench file's top-level `switchbox` entry.

  The bench file is `tinyg_hil.benches`: `$TINYG_BENCHES` or `~/.config/tinyg-hil/benches.json`, kept outside the repository. The tracked `config/benches.json` is only an example with placeholders. The port is never guessed.
- The client refuses TinyG bench ports and `never_open` ports.
- The `switchbox` fixture:
  - releases every contact and idles every channel before the test, and releases after;
  - records the test's switchbox traffic and board events in the run record;
  - its session fixture holds the board lock for the switchbox's port and records the board's identity in `run.json` (`instruments.switchbox`).
- `close/open/pulse/release_all` have the `SoftSwitches` meaning (close = trip for the channel's type), so a test written against the ICE `switches` fixture can take `switchbox` instead. Make the channel types match `$st`.

## Limits and timing

- **Never time edges from the host.**
  - A serial command costs about 87 µs per byte at 115200 (a typical request plus reply is 10-40 ms on the wire), plus USB-serial latency of 1-16 ms each way.
  - `pulseChannel` and `bounceChannel` time both edges on the board.
- **On-board timing:**
  - Edges closer than 50 µs (short pulses, fast bounces) are timed by spinning on the µs clock with interrupts masked: about ±1 µs. Interrupts stay masked for the whole sequence, at most 64 × 50 µs = 3.2 ms.
  - Longer gaps use the timer1 interrupt (0.2 µs ticks). Entry latency is a few µs; WiFi can delay it by tens of µs now and then. Bounce edges are scheduled on absolute times, so latency does not accumulate.
  - Every edge's actual timestamp is in the event log, so a test can check the widths it got.
  - These are the board's own timestamps. Any timing claim about what TinyG saw still needs a Saleae capture on the TinyG input (bench 2 has X min on ch0). Until then it is timing-pending.
- **Line watch:** polled every 10 ms; shorter external glitches are not seen.
- **Event ring:** 128 events. `getEvents` returns at most 32 per page; `lost` says how many were overwritten.
- **Ranges:** 8 channels; `ttl_ms` ≤ 1 h; pulses ≤ 60 s (`ms`) or ≤ 1 s (`us`); bounces ≤ 64 edges, 2 µs-100 ms apart.
- **Heap:** about 40 KB of usable heap. Large content (spec ~26 KB, UI) is served from flash in chunks. `getSpec` is not available over the WebSocket.

## Possible later feature: "mount" (not implemented)

Emulate a switch fixed at a machine position, without the Atmel-ICE: count the TinyG's step/dir pulses for one motor on two input GPIOs, and trip a channel when the count crosses a set position. The API has room for it: a `mountChannel` op is reserved in the spec's `x-future`, and channels already carry a type and state that a mount would drive through the same sequencer.

The ESP8266 constrains it:
- **No hardware pulse counter** (the ESP32 has PCNT). Every step edge would be a GPIO interrupt. The core's dispatch costs roughly 2-4 µs at 160 MHz, which puts the ceiling around 100-200 kHz of edges in theory.
- **WiFi masks interrupts** for tens of µs at a time (longer during association and scans), so steps are missed well below that ceiling. TinyG steps at up to about 50 kHz per motor.
- Realistic use: homing-speed step rates of a few kHz, with WiFi off.
- For full-speed moves: an ESP32 (PCNT), or the existing test-hooks mount in TinyG itself (`switches.mount` with `--ice`).

## Tests

| test | needs | covers |
|---|---|---|
| `tests/host/test_switchbox_core.py` | `c++`, ArduinoJson (pinned; from `.pio/libdeps` or fetched by `pio`) | the core under ASan/UBSan with faked pins, clock and timer |
| `tests/hil/tests/test_switchbox_offline.py` | nothing | client framing, reset check, ports, the spec ↔ client parity, the firmware's embedded spec |
| `tests/hil/tests/test_switchbox.py` | the Feather on USB, `--switchbox` | the real board: readback, ttl, pulses, bounce, config validation, errors, events |
