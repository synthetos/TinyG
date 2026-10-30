# Configuration layout 2

This integration uses `CONFIG_NVM_LAYOUT=2`, marker `10002`. Layout-1 markers
`10001` and legacy `449.01` migrate without resetting existing settings: load the
unchanged prefix through `se29`, initialize and persist only the active appended
settings, then write marker `10002`. The boundary is looked up by the `spep` token.
Already-current layout-2 settings are loaded without rewriting the marker.
Erased EEPROM and unknown layouts still load profile defaults.

Keep a backup when changing firmware. Downgrading to a firmware that does not
understand the newer marker can still reset or misinterpret settings; this
forward migration cannot change older firmware's behavior.

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

The three E060 entries are placeholders at their final table indexes. They have
no initialization or persistence flags: neither reads/writes nor loading defaults
writes their EEPROM slots. Direct GETs and SETs return status 17
(`STAT_FUNCTION_IS_STUBBED`); group reports omit them. They allocate no runtime
backing fields and do **not** enable spindle/coolant pause or a resume delay.
The table's default constants are for the future implementation only; they do not
initialize EEPROM in this build. When E060 lands, replace the accessors/flags at
these same indexes and initialize its settings once (spph=1, spde=1.0, coph=0 for
the default profile). Existing contents of unused EEPROM slots cannot be assumed
to be erased or valid; E060 must establish its defaults before loading them.
No additional layout bump is needed if indexes stay fixed. The session-only `os`
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

## Output levels during boot

The GPIO1 pins used for spindle enable/direction and coolant remain inputs while
settings load. `spindle_init_outputs()` latches the configured spindle OFF level
before enabling the output drivers; driving a fixed low level earlier would
briefly enable an active-low spindle. This deliberately leaves an input/high-
impedance interval during boot. External pull resistors or the connected device
must establish its inactive level during that interval; the firmware does not
prove that a particular board has suitable pulls. Startup pulse/glitch behavior
requires external capture. The logic ordering is host-tested.

## Migration verification

`tests/host/test_nvm_migration.py` runs the actual config initializer with a
synthetic EEPROM: both legacy markers retain non-default motor geometry, offsets
and switch settings, new active rows get defaults, placeholders stay untouched,
and the migration marker is written once. A second boot preserves new settings.
Unknown markers take the existing default path.

The optional board test `test_e022_layout2_migration.py` needs
`TINYG_LAYOUT1_SOURCE` pointing at a layout-1 checkout and `--allow-destructive`.
It backs up settings, flashes layout 1, restores them, changes `1sa` to 0.9, flashes
layout 2 and verifies the preserved value plus new defaults. Its cleanup restores
the layout-2 image and saved settings. Board validation remains pending.
