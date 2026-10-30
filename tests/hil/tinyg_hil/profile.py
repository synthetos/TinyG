"""Statistical CPU profile over the Atmel-ICE: sample the program counter and attribute it to functions.

Each sample halts the CPU for a few milliseconds, so this measures where time goes on average, not
timing jitter. Interrupt handlers (avr-gcc's __vector_N) are named from avr-libc's device header and
described from the board file ("firmware_isrs").
"""
import collections
import pathlib
import re
import subprocess
import time


def _vector_names(board):
    header = board.data["mcu"].get("avr_libc_header")
    if not header:
        return {}
    inc = pathlib.Path(subprocess.run(["avr-gcc", "-print-file-name=include"], capture_output=True,
                                      text=True).stdout.strip())
    for root in (inc, *inc.parents):
        path = root / "avr" / "include" / header if not (root / header).exists() else root / header
        if path.exists():
            text = path.read_text(errors="replace")
            return {int(n): name for name, n in re.findall(r"#define (\w+)_vect_num\s+(\d+)", text)}
    return {}


class Profiler:
    def __init__(self, ice):
        self.ice = ice
        self.vectors = _vector_names(ice.board)
        self.isrs = ice.board.data.get("firmware_isrs", {})

    def _name(self, func):
        base = func.split("+", 1)[0]
        m = re.match(r"__vector_(\d+)$", base)
        if m:
            vec = self.vectors.get(int(m.group(1)), base)
            return f"ISR {vec}" + (f" ({self.isrs[vec]})" if vec in self.isrs else "")
        return base

    def sample(self, n=200, interval=0.0):
        """Counter of function name -> samples."""
        counts = collections.Counter()
        for _ in range(n):
            counts[self._name(self.ice.where())] += 1
            if interval:
                time.sleep(interval)
        return counts

    @staticmethod
    def summary(counts, top=12):
        total = sum(counts.values()) or 1
        isr = sum(v for k, v in counts.items() if k.startswith("ISR "))
        return {"samples": total, "isr_share": round(isr / total, 3),
                "top": [(name, round(v / total, 3)) for name, v in counts.most_common(top)]}
