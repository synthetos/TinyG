"""Generate the terminal-block FeatherWing carrier profile from Adafruit's Eagle files.

    python3 -I tools/carrier_from_eagle.py > boards/carrier/terminal-block-featherwing.json

Reads (never executes) docs/adafruit-terminal-block-featherwing/*.sch and *.brd (CC BY-SA 3.0):
- the schematic's nets give, for every screw terminal pin (X1..X12), the FeatherWing header pin it is
  wired to: JP3 = the Feather's 16-pin header ("L1".."L16"), JP1 = its 12-pin header ("S1".."S12");
- the board gives the terminal pad positions, the header pin positions, the Feather footprint, the
  slide switch and the board outline;
- the silkscreen (layer 21 tPlace) gives the label printed at each terminal; a terminal without a
  label within 1 mm along the row has "silk": null.
Coordinates are Eagle's: millimetres, origin at the board centre, y pointing up. Terminal ids run
left to right with the Micro-USB end on the left: T1..T16 along the SCL/SDA edge (Eagle y > 0),
B1..B16 along the RX/TX edge.
"""
import json
import math
import pathlib
import sys
import xml.etree.ElementTree as ET

HERE = pathlib.Path(__file__).resolve().parent.parent
SRC = HERE / "docs" / "adafruit-terminal-block-featherwing"
SCH = SRC / "Adafruit Terminal FeatherWing.sch"
BRD = SRC / "Adafruit Terminal FeatherWing.brd"

# FeatherWing symbol pin names (MS1) -> the Feather's own pin names
FEATHER_NAMES = {"NC": "NC", "GPIOTX": "TX", "GPIORX": "RX", "GPIOMISO": "MISO", "GPIOMOSI": "MOSI",
                 "GPIOSCK": "SCK", "GPIOA5": "A5", "GPIOA4": "A4", "GPIOA3": "A3", "GPIOA2": "A2",
                 "GPIOA1": "A1", "GPIOA0": "A0", "GND": "GND", "AREF": "AREF", "3V": "3V", "!RESET": "RST",
                 "VBAT": "BAT", "EN": "EN", "USB": "USB", "GPIO13": "13", "GPIO12": "12", "GPIO11": "11",
                 "GPIO10": "10", "GPIO9": "9", "GPIO6": "6", "GPIO5": "5", "GPIOSCL": "SCL", "GPIOSDA": "SDA"}


def r2(v):
    return round(v + 0.0, 2)


def place(element, px, py):
    rot = element.get("rot", "R0")
    a = math.radians(float(rot.lstrip("MR") or 0))
    if rot.startswith("M"):
        px = -px
    x, y = float(element.get("x")), float(element.get("y"))
    return x + px * math.cos(a) - py * math.sin(a), y + px * math.sin(a) + py * math.cos(a)


def nets():
    """(part, pin) -> (net name, the JP1/JP3 header position on that net, the Feather pin name)."""
    root = ET.parse(SCH).getroot()
    out = {}
    for net in root.iter("net"):
        refs = [(p.get("part"), p.get("pin")) for p in net.iter("pinref")]
        header = [f"{'L' if part == 'JP3' else 'S'}{pin}" for part, pin in refs if part in ("JP1", "JP3")]
        name = next((FEATHER_NAMES[pin] for part, pin in refs if part == "MS1"), None)
        for ref in refs:
            out[ref] = (net.get("name"), header, name)
    return out


def main():
    root = ET.parse(BRD).getroot()
    packages = {(lib.get("name"), p.get("name")): p for lib in root.iter("library") for p in lib.iter("package")}
    elements = {e.get("name"): e for e in root.iter("element")}
    plain = root.find(".//board/plain")
    silk = [(float(t.get("x")), float(t.get("y")), t.text) for t in plain.iter("text") if t.get("layer") == "21"]
    xs = [float(w.get(k)) for w in plain.iter("wire") if w.get("layer") == "20" for k in ("x1", "x2")]
    ys = [float(w.get(k)) for w in plain.iter("wire") if w.get("layer") == "20" for k in ("y1", "y2")]
    netmap = nets()

    terminals = []
    for name in sorted((n for n in elements if n.startswith("X")), key=lambda n: int(n[1:])):
        e = elements[name]
        pkg = packages[(e.get("library"), e.get("package"))]
        for pad in pkg.iter("pad"):
            x, y = place(e, float(pad.get("x")), float(pad.get("y")))
            net, header, fname = netmap[(name, pad.get("name"))]
            assert len(header) == 1, (name, pad.get("name"), header)
            edge = 23.0 if y > 0 else -23.0
            label = [t for tx, ty, t in silk if abs(tx - x) < 1.0 and abs(ty - edge) < 1.5]
            terminals.append({"block": name, "pin": int(pad.get("name")), "x": r2(x), "y": r2(y),
                              "edge": "scl_sda" if y > 0 else "rx_tx",
                              "silk": label[0] if label else None, "feather": header[0], "feather_name": fname,
                              "net": net,
                              "source": f"sch net {net} ({name}.{pad.get('name')} - {'JP3' if header[0][0] == 'L' else 'JP1'}."
                                        f"{header[0][1:]}); brd pad ({r2(x)}, {r2(y)})"
                                        + (f"; silkscreen '{label[0]}'" if label else "; no silkscreen label")})
    for edge, prefix in (("scl_sda", "T"), ("rx_tx", "B")):
        row = sorted((t for t in terminals if t["edge"] == edge), key=lambda t: t["x"])
        for i, t in enumerate(row, 1):
            t["id"] = f"{prefix}{i}"
            t["position"] = i
    terminals.sort(key=lambda t: (t["id"][0] != "T", t["position"]))

    headers = []
    for part, prefix in (("JP3", "L"), ("JP1", "S")):
        e = elements[part]
        for pad in packages[(e.get("library"), e.get("package"))].iter("pad"):
            x, y = place(e, float(pad.get("x")), float(pad.get("y")))
            pos = f"{prefix}{pad.get('name')}"
            fname = next(n for (p, pin), (_, h, n) in netmap.items() if p == part and pin == pad.get("name"))
            headers.append({"pos": pos, "name": fname, "x": r2(x), "y": r2(y)})
    headers.sort(key=lambda h: (h["pos"][0], int(h["pos"][1:])))

    ms1 = elements["MS1"]
    sw = elements["S2"]
    holes = [{"x": r2(float(e.get("x"))), "y": r2(float(e.get("y"))), "d": 3.0}
             for n, e in sorted(elements.items()) if e.get("package", "").startswith("MOUNTINGHOLE")]
    profile = {
        "kind": "carrier",
        "id": "terminal-block-featherwing",
        "name": "Adafruit Terminal Block Breakout FeatherWing",
        "product": "https://www.adafruit.com/product/2926",
        "sources": {
            "schematic": "docs/adafruit-terminal-block-featherwing/Adafruit Terminal FeatherWing.sch",
            "board": "docs/adafruit-terminal-block-featherwing/Adafruit Terminal FeatherWing.brd",
            "licence": "CC BY-SA 3.0, Adafruit Industries (docs/adafruit-terminal-block-featherwing/license.txt)",
            "generated_by": "tools/carrier_from_eagle.py",
        },
        "units": "mm, Eagle board frame: origin at the board centre, y up; Micro-USB end at -x",
        "outline": {"x0": r2(min(xs)), "y0": r2(min(ys)), "x1": r2(max(xs)), "y1": r2(max(ys)), "corner": 2.54},
        "edges": {"scl_sda": "the edge whose silkscreen reads 3v3 3v3 GND GND BAT Ena USB ... SCL SDA",
                  "rx_tx": "the edge whose silkscreen reads Rst 3v3 GND ... SCK MOSI MISO RX TX"},
        "terminal_pitch": 3.5,
        "feather": {"x0": r2(float(ms1.get("x"))), "y0": r2(float(ms1.get("y"))), "width": 50.8, "height": 22.86,
                    "usb_end": "-x", "headers": headers},
        "switch": {"x": r2(float(sw.get("x"))), "y": r2(float(sw.get("y"))), "label": "3v Enable", "net": "ENABLE",
                   "function": "slide switch from the Feather's EN pin to GND: OFF turns the Feather's 3.3 V regulator off",
                   "source": "sch S2 (O = ENABLE, P = GND); silkscreen '3v Enable', 'ON', 'OFF'"},
        "holes": holes,
        "terminals": terminals,
    }
    json.dump(profile, sys.stdout, indent=1, ensure_ascii=False)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
