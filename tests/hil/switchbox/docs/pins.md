# Which ESP8266 pins can be switchbox channels

Question: the ESP8266 has 17 GPIOs, so why does the switchbox use only GPIO 4, 5, 12, 13, 14, 16, 0 and 2?

Short answer: those are the only pins that can emulate a contact to a pulled-up TinyG input while the USB serial port keeps working:
- Of the 17 GPIOs, six (6-11) run the module's SPI flash and do not even have a pad on the Feather.
- Two (1, 3) are the USB serial port.
- GPIO15 has a 4.7 kΩ boot-strap pull-down that would hold the TinyG input low.
- That leaves the 8 in use. They map one-to-one onto the TinyG v8's 8 switch inputs.

## Sources

- **Netlist:** Adafruit's Eagle schematic `adafruit-pcb/Adafruit ESP8266 Feather rev G.sch` (CC BY-SA 3.0). Part and net names below are from that file.
- **Pin functions and boot straps:**
  - Espressif ESP8266EX datasheet v7.1 (`local/`), Table 2-1 and its notes: "GPIO2, GPIO0, and MTDO are used to select booting mode", and "U0TXD should not be pulled externally to a low logic level during the powering-up".
  - The standard ESP8266 boot-mode table: (MTDO/GPIO15, GPIO0, GPIO2) = (0, 1, 1) boots from flash; (0, 0, 1) is UART download.
- **Feather silkscreen and module:** the pinout PDF v1.2 (`local/`). The module is an Ai-Thinker ESP8266MOD (ESP-12F footprint).
- **TinyG side:**
  - Inputs: X/Y/Z/A min/max on ports A, D, E and F, bits 6/7, with the internal pull-up (`firmware/tinyg/switch.c`, `tests/hil/boards/tinyg-v8.json`). That is 8 inputs; the probe is one of them (`SW_MODE_PROBE`).
  - The harness's board file also notes that the v8 inputs "have pull-ups the internal pull-down cannot overcome".

## Per pin (Feather HUZZAH ESP8266, Rev G)

"Header" gives the Feather silkscreen label. The Feather's long header is JP1 and the short one is JP3.

| ESP pin | module pad | Feather header | also wired to on the Feather | boot strap | channel to a pulled-up TinyG input? | switchbox |
|---|---|---|---|---|---|---|
| GPIO0 | 12 | yes: `0` (JP3.8) | Q2 collector: the auto-reset transistor pulls it low whenever DTR is asserted without RTS. Red LED D3 plus 4.7 kΩ (R7) from 3.3 V (lit while low) | must be **high** at boot (low = UART download) | **yes, with caveats**: a closed contact or real switch during ESP reset leaves it in the bootloader; flashing and some serial tools pull it low (spurious trip) | `amin` |
| GPIO1 (U0TXD) | 16 | yes: `TX` (JP1.2) | CP2104 RXD, direct | must not be pulled low at power-up; the ROM prints its boot log here | **no** while USB serial is the control link: it is the UART TX, so every reply would toggle the TinyG input | — |
| GPIO2 | 11 | yes: `2` (JP3.10) | 10 kΩ pull-up (R2) to 3.3 V. The blue LED is on the ESP-12 module itself, not on the Feather schematic. Also UART1 TX | must be **high** at boot | **yes, with caveats**: a closed contact or real switch during ESP reset stops it booting | `amax` |
| GPIO3 (U0RXD) | 15 | yes: `RX` (JP1.3) | CP2104 TXD through 1 kΩ (R11) | — | **no**: the CP2104 drives it (idle high, toggling with host traffic) | — |
| GPIO4 | 13 | yes: `4`/SDA (JP3.12) | nothing | none | **yes** | `xmin` |
| GPIO5 | 14 | yes: `5`/SCL (JP3.11) | nothing | none | **yes** | `xmax` |
| GPIO6-11 | none on the Feather | **no** | inside the module: the 4 MB SPI flash (SDIO_CLK/DATA0-3/CMD). The Feather's ESP-12F footprint has only the 16 edge pads, not the module's six bottom-edge pads | — | **no**: unreachable, and in use running the program | — |
| GPIO12 | 6 | yes, twice: `12` (JP3.5) and `MI` (JP1.4), same net | nothing | none | **yes** | `ymin` |
| GPIO13 | 7 | yes, twice: `13` (JP3.6) and `MO` (JP1.5) | nothing | none | **yes** | `ymax` |
| GPIO14 | 5 | yes, twice: `14` (JP3.4) and `SCK` (JP1.6) | nothing | none | **yes** | `zmin` |
| GPIO15 (MTDO) | 10 | yes: `15` (JP3.7) | **4.7 kΩ pull-down** (R7) to GND | must be **low** at boot | **no, not directly**: the 4.7 kΩ pull-down and TinyG's pull-up form a divider (below). It works through an N-MOSFET; see option 1 | — |
| GPIO16 (XPD_DCDC) | 4 | yes: `16` (JP3.9) | nothing. It is not tied to RST; deep-sleep wake would need a jumper, which would make closing this channel reset the ESP | none | **yes**. It has no pin interrupt and only an internal pull-down, which is irrelevant here; the firmware drives it through its own registers | `zmax` |
| ADC (TOUT) | 2 | yes: `A` (JP1.12) | nothing (0-1.0 V input, no divider on the Feather) | — | **no**: analog input only, no digital output | — |
| CH_PD (CHIP_EN) | 3 | yes: `CHPD` (JP1.1) | 10 kΩ pull-up (R2) | must be high to run | **no**: chip enable, not a GPIO (low powers the ESP down) | — |
| RST (EXT_RSTB) | 1 | yes: `RST` (JP1.16) | 10 kΩ pull-up (R3), the Feather's only push button SW2, Q1 collector (RTS auto-reset) | — | **no**: not a GPIO | — |
| (`EN` header pin) | — | yes: `EN` (JP3.2) | the 3.3 V regulator's enable (10 kΩ pull-up R1), not the ESP | — | **no**: it switches the Feather's 3.3 V off | — |

**Count:**
- 11 digital GPIOs reach the headers: 0, 1, 2, 3, 4, 5, 12, 13, 14, 15, 16.
- With the USB serial kept, 1 and 3 are taken, and GPIO15 cannot be used directly.
- That leaves exactly the 8 the firmware uses. The default map puts the boot-strap pins 0 and 2 last (`amin`/`amax`).

### Why GPIO15 cannot be wired straight to a TinyG input

An "open" channel must leave the line to TinyG's pull-up. On GPIO15 the Feather's 4.7 kΩ pull-down (R7) stays connected, so the open line settles at a divider voltage:

| TinyG pull-up | open line | TinyG reads | ESP at boot (VIL ≤ 0.25 × 3.3 V = 0.83 V) |
|---|---|---|---|
| xmega internal only (tens of kΩ, ~24 kΩ typical) | ≈ 0.5 V | low = **switch always closed** | boots |
| internal plus a v8 board pull-up (e.g. 10 kΩ in parallel, ≈ 7 kΩ) | ≈ 1.3 V | undefined | **above VIL: the boot strap may read high and the ESP fails to boot from flash** |

Either way the channel can never present "open". This is the schematic-level reason it is refused (`gpio_forbidden`). The exact v8 pull-up value is not in this repository; both cases fail.

### The breakout board

Nothing available identifies the breakout board the Feather is mounted on: it has no USB presence, and none of these documents cover it. One electrical hint: with nothing wired, the unwired channel pins (4, 5, 12, 13, 14, 16) read a steady low, while 0 and 2 read high (from the Feather's own LED path and pull-up). So the breakout adds no pull-ups to the signal pins. It may be a plain passive breakout, or one with pull-downs or LEDs to GND. A photo or the board's marking would settle it.

## Can there be more than 8 channels?

The TinyG v8 has exactly 8 switch inputs, and the switchbox already covers all of them. Extra channels would only serve other targets, such as a second controller or spare test signals.

| option | channels | cost | trade-offs |
|---|---|---|---|
| 1. **GPIO15 through an N-MOSFET** (2N7002/BSS138: gate on GPIO15, drain to the TinyG input, source to GND) | +1 → 9 | one transistor (plus an optional gate resistor). Firmware needs a per-channel "drive high to close" mode | Boot-safe by construction: GPIO15 is low at boot, so the FET is off and the contact open. The FET isolates the strap from TinyG's pull-up. Timing is unchanged (plain GPIO, works from the timer interrupt). It cannot be used on 0/2: they boot high, which would close the contact during every ESP boot |
| 2. **RX/TX (GPIO3/1) as channels** | +2 → 10 | the USB serial control link has to go (WiFi only) | **Not recommended**: <ul><li>loses the WiFi-independent control, provisioning and recovery path the harness relies on;</li><li>GPIO1 carries the ROM boot log (edges on every ESP reset) and must not be low at power-up;</li><li>GPIO3 is driven by the CP2104 through 1 kΩ whenever USB is plugged in, and toggles with host traffic and flashing.</li></ul>`Serial.swap()` moves UART0 to GPIO13/15, which costs a working channel and gains nothing |
| 3. **I2C port expander on GPIO4/5** | MCP23017: 8 − 2 + 16 = **22** per chip (up to 8 chips). PCF8574: 8 − 2 + 8 = 14 | one chip, plus I2C pull-ups (the expander's), plus wiring | Both power up released: MCP23017 pins come up as inputs; PCF8574 comes up as quasi-high with a weak ~100 µA source, so it is not truly high-Z and back-feeds a little into an unpowered TinyG. The MCP23017 can do the same open-drain emulation by toggling IODIR. But each I2C write takes ~25-100 µs at 400 kHz and cannot run from the timer interrupt, so pulses and bounces on expander channels lose their µs timing (loop-timed, ms jitter). Readback costs bus time |
| 4. **Move to an ESP32 Feather (e.g. HUZZAH32)** | about 20 usable GPIOs | board swap, plus porting the ESP8266 layer (`src/main.cpp`; `lib/sbcore` is portable) | Keeps µs timing on every channel. Adds hardware pulse counters (PCNT), which the planned step-counting "mount" feature needs (README). Best if the switchbox has to grow |

**Recommendation:**
- **Keep the 8 direct channels.** They cover every TinyG v8 switch input with µs-timed edges, and nothing in the schematic argues for changing the pin map.
- If one more channel is ever needed, use **GPIO15 through an N-MOSFET** (option 1). It is cheap, boot-safe and keeps the timing.
- If many more are needed, or the "mount" feature is wanted, **move to an ESP32 Feather** (option 4) rather than an I2C expander. The expander would trade away the µs edge timing that is the switchbox's reason to exist.
- Using RX/TX (option 2) is not worth losing the USB control link.
