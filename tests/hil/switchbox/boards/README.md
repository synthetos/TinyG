# Board profiles

The switchbox's hardware is described as data, so that a different Feather or carrier board is a new file rather than new code. One source of truth feeds:
- the firmware build (`firmware/scripts/embed_assets.py`), which serves it at `GET /api/board` (serial op `getBoard`) and draws the web UI's Wiring view from it;
- `tools/render_pinout.py`, which draws `docs/wiring.svg`/`.png` and writes `docs/wiring.md`;
- the offline tests (`tests/hil/tests/test_switchbox_boards_offline.py`) and the host test (`tests/host/test_switchbox_core.py`).

| file | describes | how it is made |
|---|---|---|
| `carrier/terminal-block-featherwing.json` | Adafruit Terminal Block Breakout FeatherWing: board outline, Feather footprint and header pins, the slide switch, and every screw terminal with its silkscreen label (or `null`), position, coordinates and the Feather header position it is wired to | **generated** from Adafruit's Eagle files by `tools/carrier_from_eagle.py`; a test checks that it still regenerates byte-for-byte |
| `feather/huzzah-esp8266.json` | Feather HUZZAH ESP8266: each Feather header position → MCU pin, GPIO number and capability, with a note and a source | hand-written from the HUZZAH Rev G schematic and the ESP8266EX datasheet (`docs/pins.md`) |
| `target/tinyg-v8.json` | TinyG v8 switch terminals J7/J8 (pin → input name) and the input circuit | hand-written from the v8h schematic in `hardware/` |
| `style.json` | capability colours (light and dark) and legend text | hand-written |

**The live channel map is not in any profile.** Which channel is on which GPIO (xmin → GPIO4 …) is the board's runtime config (`getConfig`/`setConfig`). The UI joins it with the profiles in the browser. `render_pinout.py` uses the firmware's default map from `default_config()` in `lib/sbcore/sbcore.cpp`.

## Feather header positions

Carrier and Feather profiles meet at Feather header positions, which are the same on every Feather:

- `L1`-`L16` is the 16-pin header. `L1` is at the TX end; `L16` is RST. In order: `L1` (NC on most Feathers), TX, RX, MISO, MOSI, SCK, A5, A4, A3, A2, A1, A0, GND, AREF, 3V, `L16` RST.
- `S1`-`S12` is the 12-pin header: BAT, EN, USB, 13, 12, 11, 10, 9, 6, 5, SCL, SDA.

The numbering matches the FeatherWing schematic's JP3/JP1 and the HUZZAH's JP1/JP3.

## Capabilities

| capability | meaning |
|---|---|
| `switch` | usable switch output |
| `bootstrap` | usable, with a boot-strap caveat (must be high at ESP boot) |
| `pulldown` | GPIO15: boot-strap pull-down, cannot present an open contact |
| `reserved` | the USB serial port (the switchbox's control link) |
| `analog` | analog input only |
| `control` | reset / enable pins; do not wire |
| `power` | supply pins |
| `gnd` | ground; the common ground to the TinyG goes here |
| `nc` | not connected on this Feather |

## Rules the tests enforce

- Every carrier terminal's Feather position exists in the Feather profile.
- Every Feather position has a known capability and a source.
- Each channel's GPIO reaches at least one terminal. When it reaches several, exactly one is the terminal to use: the one with a silkscreen label, else the lowest id. The others carry `same_as`.
- Example: on the HUZZAH ESP8266, GPIO12/13/14 reach both the unlabelled 12/11/13 terminals and the labelled MISO/MOSI/SCK ones, so MISO/MOSI/SCK are used.
- A channel on a capability other than `switch`/`bootstrap` is flagged as unsafe.
- The firmware build fails if the env's profiles do not validate.
- The Feather profile agrees with the firmware's pin-validation rules: its `switch`/`bootstrap` GPIOs are exactly the ones `setConfig` accepts, and `gpio_caveat()` is set exactly for `bootstrap`.

## Adding a Feather (e.g. the HUZZAH32)

1. Write `feather/<id>.json` with all 28 positions, each with a `source`. Set `platformio_env`.
2. Add a PlatformIO env in `firmware/platformio.ini` with `custom_feather_profile = <id>`. There is a commented slot for `huzzah32`.
3. For a different MCU (ESP32), port `firmware/src/main.cpp` and the pin rules in `lib/sbcore` (`gpio_forbidden`/`gpio_caveat`), and extend the host test's profile comparison to that build.
4. Render the docs for it: `python3 tools/render_pinout.py --feather <id>`.

## Regenerating the carrier profile

```
cd tests/hil/switchbox
python3 -I tools/carrier_from_eagle.py > boards/carrier/terminal-block-featherwing.json
```
