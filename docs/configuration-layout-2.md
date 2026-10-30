# Configuration layout 2

This integration makes one deliberate EEPROM compatibility change:
`CONFIG_NVM_LAYOUT=2`, marker `10002`. A board with layout 1, an old build-number
marker, or erased EEPROM loads defaults once. Subsequent builds with layout 2
retain settings. Save and restore the machine configuration around the upgrade;
verify motor geometry and switch polarity before motion. Flashing old firmware
back can reset or misinterpret the newer layout, too.

All existing entries through `se29` retain their indexes. The additions follow
in this fixed order, before group entries:

| Token | Role | Initial value |
|---|---|---|
| `spep` | Persisted spindle enable polarity; 0 active-low, 1 active-high | Profile value (default 1) |
| `spdp` | Persisted spindle direction polarity; clockwise level 0/1 | Profile value (default 0) |
| `spph` | Reserved for spindle pause on hold (E060) | Profile value (default 1) |
| `spde` | Reserved for spindle spin-up delay, seconds (E060) | Profile value (default 1.0) |
| `coph` | Reserved for coolant pause on hold (E060) | Profile value (default 0) |
| `prbp` | Persisted probe polarity; 0 active-low/NO, 1 active-high/NC | 0, preserving edge probe behavior |
| `fstk` | Read-only observed unused stack margin in bytes | Measured since boot |
| `adc3` | Read-only signed raw ADCB/PB3 count | Measured on request |

The three E060 entries reserve and initialize their EEPROM slots. Their GETs
return null and SETs return read-only until the E060 runtime implementation passes
its external timing gate. They allocate no runtime backing fields in this build.
They do **not** enable automatic spindle/coolant pause or a resume delay yet.
When E060 lands, replace the accessors in these same rows, retaining their order,
types and defaults; no additional layout bump is needed. The session-only `os`
row from E020 belongs after these entries if merged afterward. Do not insert it
before them once layout 2 has been deployed.

`fstk` paints RAM after static storage at startup and reports the lowest observed
unmodified margin below the stack. Interrupts or bytes coincidentally equal to the
paint pattern can hide prior use; this is diagnostic evidence, not a proof of the
worst-case stack bound. It does not replace stress testing of homing, probing and
nested interrupts.

`adc3` and M105 use PB3 (SS2 connector, J13 pin 2 on the v8 board), single-ended
signed 12-bit mode and VCC/1.6 reference (approximately 2.06 V at VCC=3.3 V).
The full result type permits -2048..2047; a normal nonnegative input uses the
positive half of that range. Counts are not temperature or calibrated volts.
Offset and gain accuracy require a known input. The A3 errata advises against
unsigned ADC operation. Do not use SPI device 2 with this build. Text M105 emits
`ADC3:<count>`; JSON M105 adds `adc3` to the response body. Conversion polling is
bounded and leaves interrupts enabled during normal command execution.

Reference: XMEGA A manual sections 25.3.3, 25.5, 25.13 and 25.16; A3 datasheet
ADC electrical characteristics and the erratum "ADC has increased INL error for
some operating conditions". The PB3/M105 interface follows commit `9520ddee`,
with signed conversion, bounded polling and JSON-aware output added here.
