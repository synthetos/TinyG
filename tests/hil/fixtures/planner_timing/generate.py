"""Regenerate the planner timing fixtures: python3 -I generate.py (writes *.nc next to this file).

Each file is short (well under 30 s on either profile), starts from X0 Y0 Z0 in G21 G90 G94 unless it says otherwise,
and states what it exercises in its first comment. Bench-only directives (;HOST-PAUSE s, ;HOLD-RESUME) are comments
to the G-code parser; the bench runner acts on them, the host replay ignores them. 21 uses an excerpt of a real CAM
program (SolidCraft output, maintainer-supplied)."""
from pathlib import Path
import math

HERE = Path(__file__).resolve().parent
HEAD = "G21 G90 G94 G17 G64\nG92 X0 Y0 Z0\n"
F = {}


def fx(name, what, body, head=HEAD):
    what = what.replace("(", "[").replace(")", "]")      # a G-code comment ends at the first )
    F[name] = f"({what})\n{head}{body}M2\n"


fx("01_long_straight_x", "long straight X moves at two feeds: cruise, accel/decel",
   "G1 X100 F600\nG1 X0 F3000\n")
fx("02_long_straight_z", "Z plunge and retract: Z's own jerk and velocity limits",
   "G1 Z-20 F300\nG1 Z0 F1000\n")
fx("03_reversals", "back-to-back reversals on X: exact-stop at each reversal",
   "".join(f"G1 X{30 if i % 2 == 0 else 0} F1500\n" for i in range(8)))
fx("04_diag_unequal", "3-axis diagonals where the slow axis limits (motor-time constraint)",
   "G1 X60 Y40 Z-10 F3000\nG1 X0 Y0 Z0\nG1 X20 Y60 Z-5 F2000\nG1 X0 Y0 Z0\n")
fx("05_tiny_collinear", "500 collinear 0.05 mm moves: segment minimum and lookahead depth",
   "".join(f"G1 X{i * 0.05:.3f} F1500\n" for i in range(1, 501)))
fx("06_tiny_high_feed", "600 zigzag 0.08 mm moves at a high F: minimum segment time and the lookahead limit",
   "".join(f"G1 X{i * 0.08:.3f} Y{(i % 2) * 0.04:.3f} F6000\n" for i in range(1, 601)))
fx("07_corners_90", "square spiral: 90 degree junctions at falling lengths",
   "".join(f"G1 X{s} Y{0}\nG1 Y{s}\nG1 X0\nG1 Y0\n".replace("G1 X", "G1 F2000 X", 1) for s in (40, 20, 10, 5, 2)))
fx("08_corners_45_135", "zigzags with 45 and 135 degree junctions",
   "G1 F2000\n" + "".join(f"G1 X{5 * i} Y{5 if i % 2 else 0}\n" for i in range(1, 12)) +
   "".join(f"G1 X{55 - 2 * i} Y{10 + (i % 2) * 5}\n" for i in range(1, 12)))
fx("09_arcs", "full circles G2/G3, radius 10 and 25",
   "G1 X10 F1500\nG2 X10 Y0 I-10 J0\nG3 X10 Y0 I-10 J0\nG1 X25\nG2 X25 Y0 I-25 J0 F3000\n")
fx("10_helix", "helix: G2 with Z travel",
   "G1 X10 F1200\nG2 X10 Y0 Z-6 I-10 J0\nG2 X10 Y0 Z-12 I-10 J0\nG0 Z0\n")
fx("11_short_arcs", "very short arcs: radius 0.5 and 1 mm (arc segment minimum)",
   "G1 X1 F1200\n" + "".join("G2 X{0:.1f} Y0 I0.5 J0\n".format(1 + 1.0 * (i + 1)) for i in range(10)) +
   "G1 X12 F800\n" + "".join("G3 X{0:.1f} Y0 I1 J0\n".format(12 + 2.0 * (i + 1)) for i in range(6)))
fx("12_units_g20", "inch program (G20): the same square in inches, then mm",
   "G20\nG1 X1 F40\nG1 Y1\nG1 X0\nG1 Y0\nG21\nG1 X25.4 F1016\nG1 Y25.4\nG1 X0\nG1 Y0\n")
fx("13_exact_stop", "G61 exact stop versus G64 continuous on the same chain",
   "G61\n" + "".join(f"G1 X{x} Y{y} F2000\n" for x, y in ((20, 0), (20, 20), (0, 20), (0, 0))) +
   "G64\n" + "".join(f"G1 X{x} Y{y} F2000\n" for x, y in ((20, 0), (20, 20), (0, 20), (0, 0))))
fx("14_feed_changes", "F changes on every move of a chain",
   "".join(f"G1 X{5 * i} F{f}\n" for i, f in enumerate((300, 1500, 600, 3000, 900, 2400, 450, 1800), start=1)))
fx("15_spindle_coolant", "S, M3/M5 and M8/M9 between moves: command boundaries",
   "M3 S8000\nG1 X10 F1500\nS9000\nG1 X20\nM8\nG1 X30\nS10000\nM9\nG1 X40\nM5\nG1 X0\n")
fx("16_dwell_midchain", "G4 dwell in the middle of a chain",
   "G1 X20 F2000\nG1 Y10\nG4 P0.5\nG1 X0\nG1 Y0\n")
fx("17_g0_vs_g1_cap", "G0 traverse (vm) versus G1 with F above fr (the cap applies)",
   "G0 X80\nG0 X0\nG1 X80 F99999\nG1 X0\n")
fx("18_z_ramps", "XZ ramps: Z's lower jerk and velocity in coordinated moves",
   "G1 X20 Z-2 F1500\nG1 X40 Z-4\nG1 X60 Z-2\nG1 X80 Z0\nG1 X0 F3000\n")
fx("19_input_gaps", "host input gaps: the host stops sending mid chain (bench only; host replay sees no gap)",
   "G1 X10 F1500\nG1 X20\n;HOST-PAUSE 1.5\nG1 X30\nG1 X40\n;HOST-PAUSE 0.5\nG1 X0\n")
fx("20_hold_resume", "feedhold mid move and resume: acceleration from rest (bench only)",
   "G1 X60 F1200\n;HOLD-RESUME 1.0 1.5\nG1 X0\n")
fx("22_rapids_mix", "rapids, feeds and plunges mixed as CAM writes them",
   "G0 Z5\nG0 X20 Y20\nG1 Z-1 F300\nG1 X40 F1200\nG1 Y40\nG0 Z5\nG0 X0 Y0\nG1 Z0 F300\n")

src = Path("/Volumes/External/projects/TinyG-work/corpus/solidcraft/solidcraft_docklight_housing_top.gcode")
if src.exists():
    lines = [l for l in src.read_text(errors="replace").splitlines()
             if l.strip() and not l.startswith("M0") and not l.startswith("T")][:1500]
    F["21_dense_cam_excerpt"] = "(dense CAM excerpt: first 1500 motion lines of a real SolidCraft program, tiny segments)\n" + \
        "\n".join(lines) + "\nM5\nM2\n"

for name, text in F.items():
    (HERE / f"{name}.nc").write_text(text)
print(len(F), "fixtures")
