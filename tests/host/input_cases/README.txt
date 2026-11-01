Parser fixture helpers
======================
Extracted from test/gcode-input-tests (8f13b128). Each finding now has its own
normal host regression; see the test docstring for its language references.

input_harness.py extracts the production normalizer, word reader, parsers,
JSON tokenizer/marshaller, numeric conversion and NV dispatch functions. Firmware
headers supply structures and status codes. The three preludes capture execution
and setting requests instead of moving a machine or writing persistent storage.

Each input is allocated with exactly one terminating NUL. The compiler enables
address, undefined-behavior and float-cast-overflow sanitizers. Compilation is
performed in class setup, outside the assertions checking parser behavior.

Run the split tests with:
  python3 -m unittest discover -s tests/host -p 'test_e1*.py'

These tests check parsing and dispatch boundaries. They do not establish board
latency, physical motion, or AVR libc behavior for every accepted numeric spelling.
