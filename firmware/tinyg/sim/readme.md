TINYG Simulator
========

TinyG simulator is designed to allow running TinyG firmware on the host computer with no real hardware attached.
It's useful for adhoc queries, debugging with GDB and running regression tests on continuous integration server.

TinyG firmware is compiled using the host gcc, and the most important I/O functions are either stubs or fakes.
For example, instead of reading from a serial port, the simulator reads from stdin. The output goes to stdout.

Right now, there's important limitation: all planned commands are pretended to be executed immediately.
In the real firmware, the planned commands are being executed in real-time using interrupts.
Current inability to simulate this behavior (may be fixed in the future), make the simulator mostly useful for parser and state transition tests.

- To build simulator: ```cd firmware/tinyg/sim && make```
- To run simulator and type the input from keyboard: ```./tinyg.elf```
- To run simulator and play gcode file: ```./tinyg.elf input.gcode```
- To debug simulator: ```gdb ./tinyg.elf [input.gcode]```

Current state:

- simulator can be run manually and inside GDB
- Travis runs simulator on all .gcode files from gcode_samples directory and checks the exit code.

Plans:

- add hand-written gcode snippets and check the real output from the simulator with LLVM FileCheck utility.
- add fuzzer tests for text and json parser to identify and fix corner cases.
- control TinyG in simulator with tgFX
- [hard] simulate time, steppers and make it possible to verify the actual movement order sent to motors.

