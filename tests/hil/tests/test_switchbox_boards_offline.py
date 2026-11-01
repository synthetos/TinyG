"""Switchbox board profiles (tests/hil/switchbox/boards) and the wiring renderer, without hardware."""
import json
import pathlib
import re
import subprocess
import sys

import pytest

SWITCHBOX = pathlib.Path(__file__).resolve().parents[1] / "switchbox"
sys.path.insert(0, str(SWITCHBOX / "tools"))
import render_pinout as rp  # noqa: E402


@pytest.fixture(scope="module")
def boards():
    return rp.load()


@pytest.fixture(scope="module")
def rows(boards):
    return rp.terminal_map(boards)


def by_id(rows):
    return {r["id"]: r for r in rows}


def test_carrier_profile_is_what_the_eagle_files_say():
    """The committed carrier profile regenerates byte-for-byte from Adafruit's .sch/.brd."""
    out = subprocess.run([sys.executable, "-I", str(SWITCHBOX / "tools/carrier_from_eagle.py")],
                         check=True, capture_output=True, text=True).stdout
    assert out == (SWITCHBOX / "boards/carrier/terminal-block-featherwing.json").read_text()


def test_carrier_terminals_and_silkscreen(boards):
    t = boards["carrier"]["terminals"]
    assert len(t) == 32 and len({x["id"] for x in t}) == 32
    unlabeled = sorted(x["id"] for x in t if x["silk"] is None)
    # the fab print: A0-A5, AREF, 13-5 and the last position by TX carry no edge label
    assert unlabeled == sorted(["T8", "T9", "T10", "T11", "T12", "T13", "T14",
                                "B3", "B5", "B6", "B7", "B8", "B9", "B10", "B16"])
    assert all(x["source"] for x in t)


def test_feather_profile_is_complete_and_cited(boards):
    pos = boards["feather"]["positions"]
    assert set(pos) == {f"L{i}" for i in range(1, 17)} | {f"S{i}" for i in range(1, 13)}
    caps = boards["style"]["capabilities"]
    for name, p in pos.items():
        assert p["capability"] in caps and p["source"], name
        assert (p["gpio"] is None) == (p["capability"] in ("power", "gnd", "nc", "control", "analog")), name
    for k, c in caps.items():
        assert re.fullmatch(r"#[0-9a-f]{6}", c["light"]) and re.fullmatch(r"#[0-9a-f]{6}", c["dark"]), k


def test_terminal_map_spot_checks(rows):
    """Positions the guide exists for: Feather pin names on the breakout are not ESP GPIO numbers."""
    t = by_id(rows)
    assert (t["T8"]["feather_name"], t["T8"]["gpio"]) == ("13", 14)
    assert (t["T12"]["feather_name"], t["T12"]["gpio"], t["T12"]["capability"]) == ("9", 0, "bootstrap")
    assert (t["T16"]["silk"], t["T16"]["gpio"]) == ("SDA", 4)
    assert (t["T11"]["gpio"], t["T11"]["capability"]) == (15, "pulldown")
    assert (t["B16"]["feather"], t["B16"]["mcu"], t["B16"]["capability"]) == ("L1", "CH_PD", "control")
    assert {t[i]["capability"] for i in ("B6", "B7", "B8", "B9", "B10", "B3")} == {"nc"}
    assert [r["id"] for r in rows if r["capability"] == "gnd"] == ["T3", "T4", "B4"]
    assert t["B14"]["capability"] == t["B15"]["capability"] == "reserved"


def test_shared_gpios_have_one_primary_terminal(rows):
    """GPIO12/13/14 reach two terminals each; the labelled one is used, the other marked same_as."""
    t = by_id(rows)
    assert (t["B11"]["silk"], t["B11"]["same_as"], t["T8"]["same_as"]) == ("SCK", None, "B11")
    assert (t["B12"]["silk"], t["T10"]["same_as"]) == ("MOSI", "B12")
    assert (t["B13"]["silk"], t["T9"]["same_as"]) == ("MISO", "B13")
    for gpio in {r["gpio"] for r in rows if r["gpio"] is not None}:
        assert len([r for r in rows if r["gpio"] == gpio and not r["same_as"]]) == 1, gpio


def test_default_channels_validate_and_land_where_documented(boards, rows):
    channels = rp.default_channels()
    assert [c for c, _ in channels] == ["xmin", "xmax", "ymin", "ymax", "zmin", "zmax", "amin", "amax"]
    assert rp.validate(boards, rows, channels) == []
    assert rp.assign(rows, channels, boards) == {
        "T16": ("xmin", "J7-3"), "T15": ("xmax", "J7-4"), "B13": ("ymin", "J7-5"), "B12": ("ymax", "J7-6"),
        "B11": ("zmin", "J8-3"), "T13": ("zmax", "J8-4"), "T12": ("amin", "J8-5"), "T14": ("amax", "J8-6")}


def test_unsafe_and_unreachable_pins_are_flagged(boards, rows):
    problems = rp.validate(boards, rows, [("p15", 15), ("tx", 1), ("flash", 7), ("ok", 4)])
    assert any("p15" in p and "pulldown" in p for p in problems)
    assert any("tx" in p and "reserved" in p for p in problems)
    assert any("flash" in p and "reaches no terminal" in p for p in problems)
    assert not any(p.startswith("channel ok") for p in problems)


def test_renderer_is_deterministic_and_docs_are_current(boards):
    channels = rp.default_channels()
    assert rp.docs_svg(boards, channels) == rp.docs_svg(boards, channels)
    assert rp.main(["--check"]) == 0, "run tools/render_pinout.py and commit docs/wiring*"


def test_every_terminal_is_drawn_and_labels_do_not_collide(boards, rows):
    asg = rp.assign(rows, rp.default_channels(), boards)
    board, _ = rp.board_svg(boards, rows, asg)
    svg = "".join(board.out)
    for r in rows:
        assert f'id="t-{r["id"]}"' in svg
    assert svg.count('class="lbl"') == 32
    assert rp.overlaps(board.boxes) == []
    width = 60
    for part, _ in (rp.tinyg_svg(boards, rows, asg, width), rp.legend_svg(boards, width)):
        assert rp.overlaps(part.boxes) == []
    frag = rp.ui_fragments(boards)
    assert set(frag) == {"board", "tinyg", "legend"} and all(f.startswith("<svg") for f in frag.values())


def test_long_channel_names_are_truncated_to_the_strip(boards, rows):
    r = by_id(rows)["B12"]
    parts = rp.fit(rp.strip_parts(r, {"B12": ("a-very-long-channel", "J7-6")}, boards))
    text = rp.SEP.join(t for t, _ in parts)
    assert len(text) <= rp.STRIP_CHARS and "…" in text


def test_board_json_is_what_getboard_serves(boards):
    b = rp.board_json(boards)
    assert set(b) == {"carrier", "feather", "target", "style", "terminals", "render"}
    assert len(b["terminals"]) == 32 and all("sources" in t for t in b["terminals"])
    assert len(json.dumps(b, separators=(",", ":"), ensure_ascii=False).encode()) < 40000
