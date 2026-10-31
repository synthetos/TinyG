"""E148: every value vector handed to mp_queue_command() holds AXES floats.

mp_queue_command() (planner.c) copies value[0..AXES-1] into the command block. A caller that passes
a local `float value[] = { x };` makes it read the stack past a one-element array (undefined
behaviour; the callbacks only use value[0], so the copied bytes are garbage, not a crash). The real
firmware sources are scanned: each local float array passed as the value argument must be declared
with AXES elements (unlisted initialisers are zero).
"""
from pathlib import Path
import re
import unittest

FIRMWARE = Path(__file__).resolve().parents[2] / "firmware/tinyg"
CALL = re.compile(r"mp_queue_command\(\s*\w+\s*,\s*(\w+)\s*,")


def short_vectors():
    found = []
    for path in sorted(FIRMWARE.glob("*.c")):
        lines = path.read_text(errors="replace").splitlines()
        for n, line in enumerate(lines):
            m = CALL.search(line)
            if not m or line.lstrip().startswith(("*", "//")):
                continue
            name = m.group(1)
            # the nearest declaration of this name above the call, within the function
            for k in range(n, max(-1, n - 40), -1):
                d = re.search(r"\bfloat\s+" + name + r"\s*\[([^\]]*)\]", lines[k])
                if d:
                    if d.group(1).strip() != "AXES":
                        found.append(f"{path.name}:{n + 1} {name}[{d.group(1).strip()}]")
                    break
    return found


class QueueCommandVectors(unittest.TestCase):
    @unittest.expectedFailure       # E148: known bug; the fix removes this marker
    def test_value_vectors_hold_axes_floats(self):
        self.assertEqual(short_vectors(), [])


if __name__ == "__main__":
    unittest.main()
