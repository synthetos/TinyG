import sys, time, collections
from tinyg_hil.switchbox import SwitchBox
sb = SwitchBox("/dev/cu.usbmodem11423101", lines="asserted", allow_reset=True, purpose="switchbox fast poll", lock_wait=30).connect()
try:
    rows = []; end = time.time() + float(sys.argv[1])
    while time.time() < end:
        st = sb.state(); rows.append((time.time(), {c["name"]: c["level"] for c in st["channels"]}))
    names = list(rows[0][1])
    print("samples", len(rows), "over", round(rows[-1][0] - rows[0][0], 2), "s")
    print("TIMES", repr([round(t, 3) for t, _ in rows]))
    for n in names:
        vals = [r[n] for _, r in rows]
        print(n, "ones", sum(vals), "of", len(vals), "changes", sum(1 for a, b in zip(vals, vals[1:]) if a != b))
finally:
    sb.disconnect()
