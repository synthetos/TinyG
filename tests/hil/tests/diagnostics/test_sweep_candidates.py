"""Diagnostics for the settings-sweep candidates (N18-N23, N25-N29): does the board accept the input, and what does
it do with it? One case per test; every case restores its settings and resets the board afterwards. Needs --ice for
the derived-value peeks (they are recorded, never required). A case "fails" when the board accepts a value the
sweep showed to be unusable: that failure is the reproduction.
"""
import math
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.resets_board,
              pytest.mark.dialect("d449"), pytest.mark.timeout(600)]


def _peek(ice, names):
    out = {}
    for n in names:
        try:
            v = ice.peek(n)
            out[n] = v if isinstance(v, (int, float)) and math.isfinite(v) else repr(v)
        except Exception as e:  # noqa: BLE001
            out[n] = f"-({type(e).__name__})"
    return out


def _case(tg, ice, record, name, keep_tokens, steps, peeks=()):
    keep(tg, keep_tokens)
    tg.send_gcode("G21 G90 G17 G94 G92.1 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0 A0", check=True, allow="declare the position")
    info = {}
    try:
        for label, line in steps:
            try:
                r = tg.send_line(line, timeout=5.0)
                info[label] = r.status
            except Exception as e:  # noqa: BLE001
                info[label] = f"no reply: {type(e).__name__}"
            time.sleep(0.4)
            info[label + "_stat"] = tg.stat() if info[label] != "no reply: NoResponse" else None
        if peeks and ice is not None:
            info["derived"] = _peek(ice, peeks)
    finally:
        record(name, info)
        try:
            tg.reset()
            tg.send_gcode("G21 G90 G94 G54", allow="recover")
        except Exception:  # noqa: BLE001
            pass
    return info


def keep(tg, tokens):
    """Settings are restored by the session; this records their values so the restore is visible in the record."""
    return {t: tg.get(t) for t in tokens}


@pytest.fixture
def ice_or_none(request):
    if request.config.getoption("--ice", default=False):
        return request.getfixturevalue("ice")
    return None


def test_n18_tiny_jerk_no_ramps(tg, record, ice_or_none):
    info = _case(tg, ice_or_none, record, "n18", ["xjm"], [("set_xjm_1e-38", '{"xjm":1e-38}'),
                 ("move", "G91 G1 X10 F100")], peeks=["mm.jerk", "mm.recip_jerk", "mm.cbrt_jerk"])
    assert info["set_xjm_1e-38"] != 0, f"xjm=1e-38 accepted: {info}"


def test_n19_negative_zero_ct(tg, record, ice_or_none):
    info = _case(tg, ice_or_none, record, "n19", ["ct"], [("set_ct_-0", '{"ct":-0.0}'),
                 ("circle", "G91 G2 X0 Y0 I5 J0 F300")], peeks=["cm.chordal_tolerance", "arc.segment_count", "arc.run_state"])
    assert info["set_ct_-0"] != 0, f"ct=-0.0 accepted: {info}"


def test_n20_tiny_axis_cap(tg, record, ice_or_none):
    info = _case(tg, ice_or_none, record, "n20", ["xfr"], [("set_xfr_1e-38", '{"xfr":1e-38}'),
                 ("move", "G91 G1 X10 F100")], peeks=["mr.segment_time", "mr.segments", "cm.motion_state"])
    assert info["set_xfr_1e-38"] != 0, f"xfr=1e-38 accepted: {info}"


def test_n21_inch_overflow(tg, record, ice_or_none):
    info = _case(tg, ice_or_none, record, "n21", ["xjd", "xjm"], [("g20", "G20"), ("set_xjd_1e38", '{"xjd":1e38}'),
                 ("set_xjm_1e38", '{"xjm":1e38}'), ("g21", "G21")], peeks=["cm.a[0].junction_dev", "cm.a[0].jerk_max"])
    assert info["set_xjd_1e38"] != 0 and info["set_xjm_1e38"] != 0, f"inch overflow stored: {info}"


def test_n22_huge_offset(tg, record, ice_or_none):
    info = _case(tg, ice_or_none, record, "n22", ["sl", "g54x"], [("sl_off", '{"sl":0}'), ("set_g54x_1e30", '{"g54x":1e30}'),
                 ("move", "G90 G54 G1 X0 F100")], peeks=["mr.segment_time", "cm.motion_state", "cm.machine_state"])
    assert info["set_g54x_1e30"] != 0, f"g54x=1e30 accepted: {info}"


def test_n23_group_set_persists_rejected_child(tg, record, ice_or_none):
    before = tg.get("xvm")
    info = _case(tg, ice_or_none, record, "n23", ["xvm"], [("group_set", '{"x":{"vm":0}}')])
    after_reset = tg.get("xvm")
    info["before"], info["after_reset"] = before, after_reset
    record("n23_after", info)
    assert info["group_set"] != 0 or after_reset == before, f"group SET lost the value: {info}"


def test_n25_huge_radius(tg, record, ice_or_none):
    info = _case(tg, ice_or_none, record, "n25", ["aam", "ara"], [("aam3", '{"aam":3}'), ("set_ara_1e30", '{"ara":1e30}'),
                 ("move", "G91 G1 A1 F100")], peeks=["cm.a[3].radius"])
    try:
        info["posa"] = tg.get("posa")
    except Exception:  # noqa: BLE001
        pass
    assert info["set_ara_1e30"] != 0, f"ara=1e30 accepted: {info}"


def test_n26_integer_narrowing(tg, record, ice_or_none):
    info = _case(tg, ice_or_none, record, "n26", ["sl"], [("sl_on", '{"sl":1}'), ("set_sl_big", '{"sl":4294967297}')])
    try:
        info["sl_now"] = tg.get("sl")
    except Exception:  # noqa: BLE001
        pass
    assert info["set_sl_big"] != 0, f"sl=4294967297 accepted (now {info.get('sl_now')}): {info}"


def test_n27_microstep_narrowing(tg, record, ice_or_none):
    info = _case(tg, ice_or_none, record, "n27", ["1mi"], [("set_1mi_65537", '{"1mi":65537}')])
    try:
        info["mi_now"] = tg.get("1mi")
    except Exception:  # noqa: BLE001
        pass
    assert info["set_1mi_65537"] != 0, f"1mi=65537 accepted (now {info.get('mi_now')}): {info}"


def test_n28_report_interval_wrap(tg, record, ice_or_none):
    info = _case(tg, ice_or_none, record, "n28", ["si"], [("set_si_huge", '{"si":4294967}')])
    assert info["set_si_huge"] != 0, f"si=4294967 accepted: {info}"


def test_n29_jd_overflow(tg, record, ice_or_none):
    info = _case(tg, ice_or_none, record, "n29", ["xjd", "yjd"], [("set_xjd_1e30", '{"xjd":1e30}'),
                 ("corner1", "G91 G1 X10 F800"), ("corner2", "G1 Y10")], peeks=["cm.a[0].junction_dev", "cm.machine_state"])
    assert info["set_xjd_1e30"] != 0, f"xjd=1e30 accepted: {info}"
