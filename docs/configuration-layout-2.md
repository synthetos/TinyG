# Configuration layout 2 (historical)

The active layout is now [layout 3](configuration-layout-3.md). Its migration
keeps the earlier machine settings and saved report selections, and removes
output-pause controls. Feedhold affects motion only.

The following hardware notes still apply.

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
