"""Switchbox on the device (Feather HUZZAH ESP8266), over USB serial; needs --switchbox.

Runs with nothing wired to the switchbox: an open (high-Z) channel then floats and may read either
level, so only driven-low readback and protocol behaviour are asserted. Edge timings here are the
board's own timestamps (micros64 at each edge): they prove the sequencing logic, and stay
timing-pending until a Saleae capture on the TinyG input confirms them.

    uv run pytest tests/test_switchbox.py --switchbox --switchbox-port /dev/cu.usbserial-XXXX \\
        --runs-dir .analysis/hil-runs
"""
import time

import pytest

from tinyg_hil.switchbox import SwitchBoxError, load_spec, redact, spec_digest

pytestmark = pytest.mark.switchbox


def edges_of(events, name, rid=None):
    return [e for e in events if e["event"] == "edge" and e.get("ch") == name and (rid is None or e.get("rid") == rid)]


def edges_after(sb, since, name):
    """Edges on `name` caused by the last request, among the board events from seq `since` on."""
    rid = sb.last_id                        # before events() sends requests of its own
    return edges_of(sb.events(since=since, all=True), name, rid)


def test_identity_and_no_reset_on_connect(switchbox, record):
    a = switchbox.info()
    time.sleep(0.2)
    b = switchbox.info()
    record("info", redact(a))
    assert switchbox.reset_on_connect is False
    assert a["boot_id"] == b["boot_id"] == switchbox.info_at_connect["boot_id"]
    assert b["uptime_us"] - a["uptime_us"] >= 200000
    spec = load_spec()
    assert a["api_version"] == spec["info"]["version"]
    assert a["spec_sha256"] == spec_digest(spec), "the board was built from a different openapi.json"


def test_spec_over_serial_matches_the_checkout(switchbox):
    assert switchbox.spec() == load_spec()


def test_driven_low_readback_on_every_channel(switchbox, record):
    seen = {}
    for c in switchbox.config()["channels"]:
        closed = switchbox.contact(c["name"], state="closed")["channel"]
        assert (closed["state"], closed["level"], closed["warning"]) == ("closed", 0, None), closed
        opened = switchbox.contact(c["name"], state="open")["channel"]
        assert opened["state"] == "open"
        seen[c["name"]] = {"gpio": c["gpio"], "closed_level": closed["level"], "open_level": opened["level"]}
    record("readback", seen)


def test_semantic_actions_follow_the_channel_type(switchbox):
    name = switchbox.config()["channels"][0]["name"]
    original = switchbox.config()["channels"]
    try:
        switchbox.set_config([dict(original[0], type="nc")] + original[1:])
        r = switchbox.trip(name)["channel"]
        assert (r["state"], r["tripped"]) == ("open", True)
        r = switchbox.release(name)["channel"]
        assert (r["state"], r["tripped"], r["level"]) == ("closed", False, 0)
        switchbox.release_all()
        assert switchbox.channel(name)["tripped"] is True          # released = open = NC tripped
        switchbox.idle()
        assert switchbox.channel(name)["state"] == "closed"
    finally:
        switchbox.set_config(original)


def test_ttl_opens_the_contact_by_itself(switchbox):
    since = switchbox.head()
    r = switchbox.contact("xmin", state="closed", ttl_ms=200)
    rid = switchbox.last_id
    assert r["channel"]["ttl_until_us"] - r["channel"]["since_us"] == 200000
    time.sleep(0.35)
    assert switchbox.channel("xmin")["state"] == "open"
    ttl = [e for e in edges_of(switchbox.events(since=since, all=True), "xmin", rid) if e["cause"] == "ttl"]
    assert len(ttl) == 1 and ttl[0]["src"] == "ttl"


@pytest.mark.timing
@pytest.mark.parametrize("kw, width_us, slack_us", [({"us": 20}, 20, 3), ({"us": 200}, 200, 25),
                                                    ({"ms": 20}, 20000, 50)])
def test_pulse_edges_are_timed_on_the_board(switchbox, record, kw, width_us, slack_us):
    since = switchbox.head()
    switchbox.pulse("x", "min", wait=False, **kw)
    time.sleep(width_us / 1e6 + 0.01)
    edges = edges_after(switchbox, since, "xmin")
    assert [e["state"] for e in edges] == ["closed", "open"]
    width = edges[1]["t_us"] - edges[0]["t_us"]
    record("pulse", {"requested_us": width_us, "board_measured_us": width})
    assert width_us <= width <= width_us + slack_us
    assert switchbox.channel("xmin")["state"] == "open"


@pytest.mark.timing
def test_bounce_sequence(switchbox, record):
    since = switchbox.head()
    r = switchbox.bounce("y", "min", edges=5, interval_us=300, final_state="closed", wait=False)
    time.sleep(0.05)
    edges = edges_after(switchbox, since, "ymin")
    assert [e["state"] for e in edges] == ["closed", "open", "closed", "open", "closed"]
    gaps = [b["t_us"] - a["t_us"] for a, b in zip(edges, edges[1:])]
    record("bounce_gaps_us", gaps)
    assert all(280 <= g <= 330 for g in gaps), gaps
    assert r["final_state"] == "closed" and switchbox.channel("ymin")["level"] == 0


def test_config_validation_rejects_unsafe_pins(switchbox):
    before = switchbox.config()
    for gpio in (15, 1, 3, 6, 11, 17):
        with pytest.raises(SwitchBoxError) as e:
            switchbox.set_config([{"name": "xmin", "gpio": gpio}])
        assert e.value.code == "invalid_config" and f"GPIO{gpio}" in e.value.msg
    assert switchbox.config() == before


@pytest.mark.persists
def test_config_round_trip_is_persisted(switchbox):
    original = switchbox.config()["channels"]
    renamed = [dict(c, name="xlo") if c["name"] == "xmin" else c for c in original]
    try:
        r = switchbox.set_config(renamed)
        assert r["persisted"] is True
        assert [c["name"] for c in switchbox.config()["channels"]] == [c["name"] for c in renamed]
    finally:
        assert switchbox.set_config(original)["persisted"] is True


def test_structured_errors(switchbox):
    for call, code in [(lambda: switchbox.close("w", "min"), "unknown_channel"),
                       (lambda: switchbox.call("explode"), "unknown_op"),
                       (lambda: switchbox.call("getState", verbose=1), "unknown_param"),
                       (lambda: switchbox.set_channel("xmin", state="closed", action="trip"), "conflicting_params"),
                       (lambda: switchbox.pulse("x", "min", ms=5, action="release"), "no_change")]:
        with pytest.raises(SwitchBoxError) as e:
            call()
        assert e.value.code == code


def test_events_are_ordered_and_stamped(switchbox):
    mark = switchbox.head()
    for _ in range(3):
        switchbox.close("x", "max")
        switchbox.open("x", "max")
    events = switchbox.events(since=mark, all=True)
    seqs = [e["seq"] for e in events]
    assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs)
    assert all(b["t_us"] >= a["t_us"] for a, b in zip(events, events[1:]))
    assert len(edges_of(events, "xmax")) == 6
    assert all(len(e["levels"]) == len(e["driven"]) for e in events)


def test_subscribed_events_arrive_as_lines(switchbox):
    first = switchbox.subscribe(True)["next"]
    try:
        switchbox.close("z", "min")
        switchbox.open("z", "min")
        switchbox.poll(0.5)
    finally:
        switchbox.subscribe(False)
        switchbox.poll(0.1)
    got = [e for e in switchbox.received if e.get("seq", 0) >= first]     # lines read during calls and polls
    assert [e["state"] for e in edges_of(got, "zmin")] == ["closed", "open"]
