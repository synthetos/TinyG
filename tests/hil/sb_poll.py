"""Poll the switchbox's line levels for SECONDS and print every change with wall-clock time (read only)."""
import sys
import time

from tinyg_hil.switchbox import SwitchBox

seconds = float(sys.argv[1])
sb = SwitchBox("/dev/cu.usbmodem3101", lines="asserted", allow_reset=True, purpose="switchbox level poll",
               lock_wait=30).connect()
try:
    names = None
    last = None
    end = time.time() + seconds
    while time.time() < end:
        st = sb.state()
        names = names or [c["name"] for c in st["channels"]]
        lv = {c["name"]: c["level"] for c in st["channels"]}
        if lv != last:
            print(round(time.time(), 3), "".join(str(lv[n]) for n in names), flush=True)
            last = lv
        time.sleep(0.1)
    print("NAMES", " ".join(names), flush=True)
finally:
    sb.disconnect()
