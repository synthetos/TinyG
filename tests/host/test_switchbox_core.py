"""ASan/UBSan host build of the switchbox firmware core (tests/hil/switchbox/firmware/lib/sbcore).

The core (request parsing and validation, channel model, edge sequencer, event ring) has no Arduino
dependency. It is compiled with the host driver test/host/sb_host.cpp, which fakes the pins, the
clock and the timer interrupt, and is fed request lines exactly as the serial port would be.

Needs ArduinoJson (the version pinned in platformio.ini): $ARDUINOJSON_INCLUDE, the firmware's
.pio/libdeps after any `pio run`, or `pio pkg install` into .pio/host-libdeps when pio is on PATH.
"""
import glob
import json
import os
import pathlib
import re
import shutil
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]
FW = ROOT / "tests/hil/switchbox/firmware"
SPEC = json.loads((ROOT / "tests/hil/switchbox/openapi.json").read_text())


def arduinojson_include():
    if os.environ.get("ARDUINOJSON_INCLUDE"):
        return os.environ["ARDUINOJSON_INCLUDE"]
    for pattern in (".pio/libdeps/*/ArduinoJson/src", ".pio/host-libdeps/ArduinoJson/src"):
        hits = glob.glob(str(FW / pattern))
        if hits:
            return hits[0]
    pin = re.search(r"bblanchon/ArduinoJson@([\d.]+)", (FW / "platformio.ini").read_text()).group(1)
    if shutil.which("pio"):
        subprocess.run(["pio", "pkg", "install", "--library", f"bblanchon/ArduinoJson@{pin}",
                        "--storage-dir", str(FW / ".pio/host-libdeps")], check=True, capture_output=True)
        hits = glob.glob(str(FW / ".pio/host-libdeps/ArduinoJson/src"))
        if hits:
            return hits[0]
    return None


class SwitchboxCore(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        inc = arduinojson_include()
        if not inc:
            raise unittest.SkipTest("ArduinoJson not found: set ARDUINOJSON_INCLUDE or run `pio run` in the firmware dir")
        cls.tmp = tempfile.TemporaryDirectory()
        cls.exe = pathlib.Path(cls.tmp.name) / "sb_host"
        subprocess.run(["c++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-g", "-fsanitize=address,undefined",
                        "-DARDUINOJSON_USE_LONG_LONG=1",
                        "-fno-sanitize-recover=all", f"-I{FW / 'lib/sbcore'}", f"-I{inc}",
                        str(FW / "lib/sbcore/sbcore.cpp"), str(FW / "test/host/sb_host.cpp"), "-o", str(cls.exe)],
                       check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_script(self, *lines, boot=True):
        script = (["@boot"] if boot else []) + [json.dumps(x) if isinstance(x, dict) else x for x in lines]
        proc = subprocess.run([str(self.exe)], check=False, input="\n".join(script) + "\n", capture_output=True, text=True)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        out = [json.loads(line) for line in proc.stdout.splitlines()]
        return out[1:] if boot else out

    def events(self, *lines):
        """Runs the script, then returns every event (paging through getEvents)."""
        out = self.run_script(*lines, {"id": 9000, "op": "getEvents", "since": 0, "limit": 32})
        page = out[-1]["result"]
        self.assertFalse(page["more"], "test produced more than one page of events")
        return out[:-1], page["events"]

    def assertError(self, reply, code):
        self.assertFalse(reply["ok"], reply)
        self.assertEqual(reply["error"]["code"], code, reply)

    # ---- spec parity ---------------------------------------------------------------------

    def test_ops_and_enums_match_the_spec(self):
        names = self.run_script("@names", boot=False)[0]
        spec_ops = {o["operationId"] for p in SPEC["paths"].values() for o in p.values()}
        spec_ops |= set(SPEC["x-serial-protocol"]["operations"])
        self.assertEqual(set(names["ops"]), spec_ops)
        schemas = SPEC["components"]["schemas"]
        self.assertEqual(names["errors"], schemas["ErrorCode"]["enum"])
        ev = schemas["Event"]["properties"]
        self.assertEqual(names["kinds"], ev["event"]["enum"])
        self.assertEqual(names["sources"], ev["src"]["enum"])
        self.assertEqual(names["causes"], ev["cause"]["enum"])
        self.assertEqual(names["wifi"], schemas["Info"]["properties"]["wifi"]["properties"]["state"]["enum"])
        self.assertEqual(names["event_ring"], SPEC["x-limits"]["event_ring"])

    def test_feather_profile_agrees_with_the_firmware_pin_rules(self):
        """boards/feather/*.json capabilities against gpio_forbidden()/gpio_caveat() in the core."""
        rules = {g["gpio"]: g for g in self.run_script("@gpios", boot=False)[0]["gpios"]}
        allowed = {g for g, r in rules.items() if not r["forbidden"]}
        profiles = sorted((ROOT / "tests/hil/switchbox/boards/feather").glob("*.json"))
        self.assertTrue(profiles)
        for path in profiles:
            prof = json.loads(path.read_text())
            if prof.get("platformio_env") != "huzzah":        # rules below are the ESP8266 build's
                continue
            usable = set()
            for pos, p in prof["positions"].items():
                g = p["gpio"]
                if g is None:
                    continue
                r = rules[g]
                with self.subTest(profile=path.name, pos=pos, gpio=g):
                    if p["capability"] == "switch":
                        self.assertIsNone(r["forbidden"])
                        self.assertIsNone(r["caveat"])
                    elif p["capability"] == "bootstrap":
                        self.assertIsNone(r["forbidden"])
                        self.assertIsNotNone(r["caveat"])
                    else:
                        self.assertIsNotNone(r["forbidden"], f"{p['capability']} pin accepted by the firmware")
                if p["capability"] in ("switch", "bootstrap"):
                    usable.add(g)
            self.assertEqual(usable, allowed, f"{path.name}: usable GPIOs differ from what setConfig accepts")

    # ---- boot, info, framing -------------------------------------------------------------

    def test_boot_releases_every_channel_and_logs_a_boot_event(self):
        pins, page = self.run_script("@pins", {"id": 1, "op": "getEvents"})
        self.assertEqual(pins["pins"], {g: "z" for g in ("0", "2", "4", "5", "12", "13", "14", "16")})
        boot = page["result"]["events"][0]
        self.assertEqual((boot["event"], boot["seq"], boot["src"], boot["driven"]), ("boot", 1, "boot", "00000000"))
        self.assertEqual(boot["boot_id"], "1234abcd")

    def test_info(self):
        r = self.run_script({"id": 7, "op": "getInfo"})[0]
        self.assertEqual(r["id"], 7)
        self.assertTrue(r["ok"])
        self.assertEqual(r["result"]["fw_version"], "0.1.0")
        self.assertEqual(r["result"]["channels"], 8)
        self.assertEqual(r["result"]["platform"], "host")

    def test_framing_errors(self):
        out = self.run_script("not json", "[1,2]", {"id": [1], "op": "getInfo"}, {"id": 3},
                              {"id": 4, "op": "explode"}, {"id": "s", "op": 5}, "@long",
                              {"id": 5, "op": "getState", "verbose": True})
        self.assertError(out[0], "bad_json")
        self.assertIsNone(out[0]["id"])
        self.assertError(out[1], "bad_request")
        self.assertError(out[2], "bad_request")
        self.assertIsNone(out[2]["id"])
        self.assertError(out[3], "missing_param")
        self.assertEqual(out[3]["id"], 3)
        self.assertError(out[4], "unknown_op")
        self.assertError(out[5], "unknown_op")
        self.assertEqual(out[5]["id"], "s")
        self.assertError(out[6], "line_too_long")
        self.assertError(out[7], "unknown_param")

    def test_read_only_queries_are_not_logged_but_failures_are(self):
        _, ev = self.events({"id": 1, "op": "getState"}, {"id": 2, "op": "getConfig"}, {"id": 3, "op": "getInfo"},
                            {"id": 4, "op": "getState", "x": 1})
        self.assertEqual([e["event"] for e in ev], ["boot", "command"])
        self.assertEqual((ev[1]["op"], ev[1]["ok"], ev[1]["error"], ev[1]["rid"]), ("getState", False, "unknown_param", 4))

    # ---- setChannel ----------------------------------------------------------------------

    def test_contact_and_semantic_set(self):
        out = self.run_script({"id": 1, "op": "setChannel", "name": "xmin", "state": "closed"}, "@pins",
                              {"id": 2, "op": "setChannel", "name": "xmin", "state": "closed"},
                              {"id": 3, "op": "setChannel", "name": "xmin", "action": "release"}, "@pins",
                              {"id": 4, "op": "setChannel", "name": "zmax", "action": "trip"}, "@pins")
        self.assertTrue(out[0]["result"]["changed"])
        ch = out[0]["result"]["channel"]
        self.assertEqual((ch["state"], ch["tripped"], ch["level"]), ("closed", True, 0))
        self.assertEqual(out[1]["pins"]["4"], "low")
        self.assertFalse(out[2]["result"]["changed"])
        self.assertEqual(out[3]["result"]["channel"]["state"], "open")
        self.assertEqual(out[4]["pins"]["4"], "z")
        self.assertEqual(out[6]["pins"]["16"], "low")     # GPIO16 uses its own registers on the board

    def test_nc_semantics(self):
        cfg = {"id": 1, "op": "setConfig", "channels": [{"name": "xmin", "gpio": 4, "type": "nc"},
                                                        {"name": "xmax", "gpio": 5}]}
        out = self.run_script(cfg, {"id": 2, "op": "setChannel", "name": "xmin", "action": "release"}, "@pins",
                              {"id": 3, "op": "setChannel", "name": "xmin", "action": "trip"}, "@pins",
                              {"id": 4, "op": "idleAll"}, "@pins", {"id": 5, "op": "releaseAll"}, "@pins")
        self.assertTrue(out[0]["ok"], out[0])
        self.assertEqual(out[1]["result"]["channel"]["state"], "closed")
        self.assertFalse(out[1]["result"]["channel"]["tripped"])
        self.assertEqual(out[2]["pins"], {"4": "low", "5": "z"})
        self.assertTrue(out[3]["result"]["channel"]["tripped"])
        self.assertEqual(out[4]["pins"], {"4": "z", "5": "z"})
        self.assertEqual(out[5]["result"]["changed"], 1)
        self.assertEqual(out[6]["pins"], {"4": "low", "5": "z"})   # idle: NC closed, NO open
        self.assertEqual(out[8]["pins"], {"4": "z", "5": "z"})     # release: nothing driven

    def test_set_parameter_errors(self):
        out = self.run_script({"id": 1, "op": "setChannel", "name": "xmin"},
                              {"id": 2, "op": "setChannel", "name": "xmin", "state": "closed", "action": "trip"},
                              {"id": 3, "op": "setChannel", "name": "wmin", "state": "closed"},
                              {"id": 4, "op": "setChannel", "state": "closed"},
                              {"id": 5, "op": "setChannel", "name": "xmin", "state": "shut"},
                              {"id": 6, "op": "setChannel", "name": "xmin", "state": "closed", "ttl": 5},
                              {"id": 7, "op": "setChannel", "name": 4, "state": "closed"},
                              {"id": 8, "op": "setChannel", "name": "xmin", "state": "closed", "ttl_ms": -1},
                              {"id": 9, "op": "setChannel", "name": "xmin", "state": "closed", "ttl_ms": 1.5},
                              {"id": 10, "op": "setChannel", "name": "xmin", "state": "closed", "ttl_ms": "5"},
                              "@pins")
        for reply, code in zip(out, ["missing_param", "conflicting_params", "unknown_channel", "missing_param",
                                     "bad_param", "unknown_param", "bad_param", "bad_param", "bad_param", "bad_param"]):
            self.assertError(reply, code)
        self.assertEqual(out[-1]["pins"]["4"], "z")        # nothing was driven by a rejected request

    # ---- ttl ------------------------------------------------------------------------------

    def test_ttl_opens_the_contact(self):
        out, ev = self.events({"id": 11, "op": "setChannel", "name": "ymin", "state": "closed", "ttl_ms": 5},
                              "@advance 4999", "@pins", "@advance 1", "@pins")
        self.assertEqual(out[0]["result"]["ttl_ms"], 5)
        self.assertEqual(out[0]["result"]["channel"]["ttl_until_us"], 1000 + 5000)
        self.assertEqual(out[2]["pins"]["12"], "low")
        self.assertEqual(out[4]["pins"]["12"], "z")
        ttl = [e for e in ev if e["event"] == "edge" and e["cause"] == "ttl"]
        self.assertEqual(len(ttl), 1)
        self.assertEqual((ttl[0]["src"], ttl[0]["rid"], ttl[0]["state"], ttl[0]["t_us"]), ("ttl", 11, "open", 6000))

    def test_ttl_clamped_and_replaced(self):
        out = self.run_script({"id": 1, "op": "setChannel", "name": "ymin", "state": "closed", "ttl_ms": 10**12},
                              {"id": 2, "op": "setChannel", "name": "ymin", "state": "closed"},
                              "@advance 4000000000", "@pins",
                              {"id": 3, "op": "setChannel", "name": "ymin", "state": "open", "ttl_ms": 5})
        self.assertEqual(out[0]["result"]["ttl_ms"], 3600000)
        self.assertEqual(out[0]["result"]["clamped"], ["ttl_ms"])
        self.assertIsNone(out[1]["result"]["ttl_ms"])      # a later set without ttl removes it
        self.assertEqual(out[3]["pins"]["12"], "low")
        self.assertIsNone(out[4]["result"]["ttl_ms"])      # ttl only applies to a closed contact

    # ---- pulse ----------------------------------------------------------------------------

    def test_short_pulse_runs_inline(self):
        out, ev = self.events({"id": 21, "op": "pulseChannel", "name": "xmax", "us": 20}, "@pins", "@status")
        res = out[0]["result"]
        self.assertEqual((res["duration_us"], res["end_us"] - res["start_us"]), (20, 20))
        self.assertEqual(res["channel"]["state"], "open")   # already back when the reply is built
        self.assertEqual(out[1]["pins"]["5"], "z")
        self.assertGreater(out[2]["spins"], 0)
        edges = [e for e in ev if e["event"] == "edge"]
        self.assertEqual([e["state"] for e in edges], ["closed", "open"])
        self.assertEqual(edges[1]["t_us"] - edges[0]["t_us"], 20)
        self.assertTrue(all(e["rid"] == 21 and e["cause"] == "pulse" for e in edges))

    def test_long_pulse_uses_the_timer(self):
        out, ev = self.events({"id": 22, "op": "pulseChannel", "name": "xmax", "ms": 10}, "@status",
                              {"id": 23, "op": "pulseChannel", "name": "xmax", "ms": 10},
                              "@advance 9999", "@pins", "@advance 1", "@pins")
        self.assertEqual(out[0]["result"]["channel"]["pending"]["kind"], "pulse")
        self.assertEqual(out[1]["armed_at"], 1000 + 10000)
        self.assertError(out[2], "busy")
        self.assertEqual(out[4]["pins"]["5"], "low")
        self.assertEqual(out[6]["pins"]["5"], "z")
        edges = [e for e in ev if e["event"] == "edge"]
        self.assertEqual(edges[1]["t_us"] - edges[0]["t_us"], 10000)

    def test_pulse_errors_and_clamps(self):
        out = self.run_script({"id": 1, "op": "pulseChannel", "name": "xmax", "ms": 5, "action": "release"},
                              {"id": 2, "op": "pulseChannel", "name": "xmax"},
                              {"id": 3, "op": "pulseChannel", "name": "xmax", "ms": 5, "us": 5},
                              {"id": 4, "op": "pulseChannel", "name": "xmax", "us": 0},
                              {"id": 5, "op": "pulseChannel", "name": "ymax", "us": 5000000},
                              {"id": 6, "op": "pulseChannel", "name": "zmin", "state": "open", "ms": 1})
        self.assertError(out[0], "no_change")
        self.assertError(out[1], "missing_param")
        self.assertError(out[2], "conflicting_params")
        self.assertEqual((out[3]["result"]["duration_us"], out[3]["result"]["clamped"]), (1, ["us"]))
        self.assertEqual(out[4]["result"]["duration_us"], 1000000)
        self.assertError(out[5], "no_change")

    def test_release_pulse_returns_to_closed(self):
        out = self.run_script({"id": 1, "op": "setChannel", "name": "zmin", "state": "closed"},
                              {"id": 2, "op": "pulseChannel", "name": "zmin", "action": "release", "ms": 2},
                              "@advance 2000", "@pins")
        self.assertEqual(out[1]["result"]["channel"]["state"], "open")
        self.assertEqual(out[3]["pins"]["14"], "low")

    # ---- bounce ---------------------------------------------------------------------------

    def test_bounce_timing_and_final_state(self):
        out, ev = self.events({"id": 31, "op": "bounceChannel", "name": "amin", "edges": 4, "interval_us": 100,
                               "final_state": "closed"}, "@advance 1000", "@pins")
        res = out[0]["result"]
        self.assertEqual((res["end_us"] - res["start_us"], res["final_state"]), (400, "closed"))
        edges = [e for e in ev if e["event"] == "edge"]
        self.assertEqual([e["state"] for e in edges], ["closed", "open", "closed", "open", "closed"])
        self.assertEqual([e["t_us"] - edges[0]["t_us"] for e in edges], [0, 100, 200, 300, 400])
        self.assertEqual(out[2]["pins"]["0"], "low")

    def test_fast_bounce_runs_inline_and_no_extra_edge(self):
        out, ev = self.events({"id": 32, "op": "bounceChannel", "name": "amax", "edges": 3, "interval_us": 10,
                               "final_action": "trip"}, "@pins")
        self.assertIsNone(out[0]["result"]["channel"]["pending"])
        edges = [e for e in ev if e["event"] == "edge"]
        self.assertEqual([e["state"] for e in edges], ["closed", "open", "closed"])   # already closed at the end
        self.assertEqual(out[1]["pins"]["2"], "low")

    def test_bounce_clamps_and_ttl(self):
        out = self.run_script({"id": 1, "op": "bounceChannel", "name": "ymax", "edges": 500, "interval_us": 1,
                               "final_state": "closed", "ttl_ms": 1}, "@advance 200000", "@pins",
                              {"id": 2, "op": "bounceChannel", "name": "ymax", "edges": 2, "interval_us": 50})
        self.assertEqual(sorted(out[0]["result"]["clamped"]), ["edges", "interval_us"])
        self.assertEqual((out[0]["result"]["edges"], out[0]["result"]["interval_us"]), (64, 2))
        self.assertEqual(out[2]["pins"]["13"], "z")        # ttl expired after the bounce settled closed
        self.assertError(out[3], "missing_param")

    def test_release_all_cancels_timers(self):
        out = self.run_script({"id": 1, "op": "pulseChannel", "name": "xmin", "ms": 50},
                              {"id": 2, "op": "setChannel", "name": "ymin", "state": "closed", "ttl_ms": 50},
                              {"id": 3, "op": "releaseAll"}, "@status", "@pins")
        self.assertEqual(out[2]["result"]["changed"], 2)
        self.assertEqual(out[3]["armed_at"], 0)
        self.assertEqual(set(out[4]["pins"].values()), {"z"})

    # ---- config ---------------------------------------------------------------------------

    def test_config_validation(self):
        def cfg(*channels):
            return {"op": "setConfig", "channels": list(channels)}
        bad = [
            (cfg({"name": "xmin", "gpio": 15}), "GPIO15"),
            (cfg({"name": "xmin", "gpio": 1}), "serial"),
            (cfg({"name": "xmin", "gpio": 3}), "serial"),
            *((cfg({"name": "xmin", "gpio": g}), "flash") for g in range(6, 12)),
            (cfg({"name": "xmin", "gpio": 17}), "not an ESP8266 GPIO"),
            (cfg({"name": "xmin", "gpio": -1}), "not an ESP8266 GPIO"),
            (cfg({"name": "xmin", "gpio": "4"}), "integer"),
            (cfg({"name": "xmin", "gpio": 4}, {"name": "xmax", "gpio": 4}), "already used"),
            (cfg({"name": "xmin", "gpio": 4}, {"name": "xmin", "gpio": 5}), "duplicate name"),
            (cfg({"name": "x min", "gpio": 4}), "name"),
            (cfg({"name": "a" * 16, "gpio": 4}), "name"),
            (cfg({"name": "xmin", "gpio": 4, "type": "NO"}), "type"),
            (cfg({"name": "xmin", "gpio": 4, "pin": 4}), "unknown key"),
            (cfg(*({"name": f"c{i}", "gpio": g} for i, g in enumerate([0, 2, 4, 5, 12, 13, 14, 16, 0]))), "1 to 8"),
            ({"op": "setConfig", "channels": []}, "1 to 8"),
            ({"op": "setConfig", "channels": {}}, "array"),
        ]
        out = self.run_script(*(b for b, _ in bad), "@pins", {"op": "setConfig"},
                              {"op": "setConfig", "defaults": False}, {"op": "setConfig", "defaults": True, "channels": []})
        for (_, needle), reply in zip(bad, out):
            self.assertError(reply, "invalid_config")
            self.assertIn(needle, reply["error"]["msg"])
        self.assertEqual(len(out[len(bad)]["pins"]), 8)    # the default mapping is untouched
        self.assertError(out[len(bad) + 1], "missing_param")
        self.assertError(out[len(bad) + 2], "bad_param")
        self.assertError(out[len(bad) + 3], "conflicting_params")

    def test_config_apply_persist_and_defaults(self):
        new = [{"name": "probe", "gpio": 13, "type": "nc"}, {"name": "zmin", "gpio": 14}]
        out, ev = self.events({"id": 1, "op": "setChannel", "name": "xmin", "state": "closed"},
                              {"id": 2, "op": "setConfig", "channels": new}, "@pins", "@savefail 1",
                              {"id": 3, "op": "getConfig"}, {"id": 4, "op": "setConfig", "defaults": True},
                              "@pins", "@status")
        res = out[1]["result"]
        self.assertTrue(res["persisted"])
        self.assertFalse(res["is_default"])
        self.assertEqual([(c["name"], c["gpio"], c["type"]) for c in res["channels"]],
                         [("probe", 13, "nc"), ("zmin", 14, "no")])
        self.assertEqual(out[2]["pins"], {"13": "z", "14": "z"})   # old pins handed back, xmin released
        self.assertEqual(out[4]["result"]["channels"], res["channels"])
        self.assertFalse(out[5]["result"]["persisted"])
        self.assertTrue(out[5]["result"]["is_default"])
        self.assertEqual(out[5]["result"]["channels"][6]["caveat"][:14], "boot-strap pin")
        self.assertEqual(len(out[6]["pins"]), 8)
        kinds = [(e["event"], e.get("cause")) for e in ev]
        self.assertIn(("edge", "config"), kinds)
        self.assertIn(("config", None), kinds)

    def test_get_config_round_trips_into_set_config(self):
        out = self.run_script({"id": 1, "op": "getConfig"})
        again = self.run_script({"id": 2, "op": "setConfig", "channels": out[0]["result"]["channels"]})
        self.assertTrue(again[0]["ok"], again[0])
        self.assertTrue(again[0]["result"]["is_default"])

    # ---- events ---------------------------------------------------------------------------

    def test_event_paging_and_overflow(self):
        flips = [{"id": 100 + i, "op": "setChannel", "name": "xmin", "state": "closed" if i % 2 == 0 else "open"}
                 for i in range(80)]              # 80 edges + 80 commands = 160 events > ring of 128
        out = self.run_script(*flips, {"id": 1, "op": "getEvents", "since": 1, "limit": 5},
                              {"id": 2, "op": "getEvents", "since": 100, "limit": 100},
                              {"id": 3, "op": "getEvents", "since": 999})
        first, clamp, future = out[-3]["result"], out[-2]["result"], out[-1]["result"]
        self.assertEqual(first["head"], 162)
        self.assertEqual(first["oldest"], 162 - 128)
        self.assertEqual(first["lost"], 162 - 128 - 1)
        self.assertEqual([e["seq"] for e in first["events"]], list(range(34, 39)))
        self.assertEqual((first["next"], first["more"]), (39, True))
        self.assertEqual(len(clamp["events"]), 32)
        self.assertEqual(out[-2]["result"]["events"][0]["seq"], 100)
        self.assertEqual((future["events"], future["next"], future["more"]), ([], 999, False))

    def test_line_watch_and_warnings(self):
        def toggle(level):
            return [f"@ext 12 {level}", "@advance 30000", "@poll"]
        out, ev = self.events("@ext 12 0", "@advance 6000", "@poll", {"id": 1, "op": "getState"},
                              *toggle(1), *toggle(0), *toggle(1), *toggle(0), *toggle(1),
                              {"id": 2, "op": "getState"},
                              *("@advance 600000", "@poll") * 4, *toggle(0), {"id": 3, "op": "getState"})
        ymin = out[3]["result"]["channels"][2]
        self.assertEqual((ymin["name"], ymin["level"]), ("ymin", 0))
        self.assertIn("reads low", ymin["warning"])
        self.assertIn("floating", out[19]["result"]["channels"][2]["warning"])
        self.assertIn("reads low", out[-1]["result"]["channels"][2]["warning"])     # quiet again
        lines = [e for e in ev if e["event"] == "line"]
        # a floating input stops being logged after 4 changes in 1-2 s, and is logged again once quiet
        self.assertEqual([e["level"] for e in lines], [0, 1, 0, 1, 0])
        self.assertGreater(lines[-1]["t_us"], 2000000)
        self.assertTrue(all(e["ch"] == "ymin" and e["src"] == "poll" and e["state"] == "open" for e in lines))

    # ---- wifi, spec, subscribe -------------------------------------------------------------

    def test_wifi_password_is_never_echoed(self):
        out, ev = self.events({"id": 1, "op": "setWifi", "ssid": "shop", "password": "hunter2hunter2"}, "@wifi",
                              {"id": 2, "op": "setWifi", "ssid": "shop", "password": "short"},
                              {"id": 3, "op": "setWifi", "ssid": "", "password": "hunter2hunter2"},
                              {"id": 4, "op": "setWifi", "ssid": "x" * 33}, {"id": 5, "op": "setWifi", "ssid": ""}, "@wifi")
        self.assertEqual(out[0]["result"], {"ssid": "shop", "cleared": False, "reconnecting": True})
        self.assertEqual(out[1], {"ssid": "shop", "password_len": 14})
        self.assertError(out[2], "bad_param")
        self.assertError(out[3], "bad_param")
        self.assertError(out[4], "bad_param")
        self.assertTrue(out[5]["result"]["cleared"])
        self.assertEqual(out[6]["ssid"], "")
        self.assertNotIn("hunter2", json.dumps(out) + json.dumps(ev))

    def test_wifi_scan_is_read_only_and_reports_busy(self):
        out, ev = self.events({"id": 1, "op": "scanWifi"}, "@scanfail 1", {"id": 2, "op": "scanWifi"},
                              {"id": 3, "op": "scanWifi", "all": True})
        self.assertEqual(out[0]["result"]["count"], 2)
        self.assertEqual([n["ssid"] for n in out[0]["result"]["networks"]], ["ExampleNet", "other"])
        self.assertError(out[2], "busy")
        self.assertError(out[3], "unknown_param")
        self.assertEqual([(e["op"], e["error"]) for e in ev if e["event"] == "command"],
                         [("scanWifi", "busy"), ("scanWifi", "unknown_param")])    # successes are not logged

    def test_spec_and_subscribe_by_transport(self):
        out = self.run_script({"id": 1, "op": "getSpec"}, {"id": 2, "op": "subscribe"}, "@src ws",
                              {"id": 3, "op": "getSpec"}, {"id": 4, "op": "subscribe", "on": False},
                              {"id": 5, "op": "subscribe"}, "@src http", {"id": 6, "op": "subscribe"},
                              {"id": 7, "op": "getInfo"})
        self.assertTrue(out[0]["stream_spec"])
        board = self.run_script({"id": 1, "op": "getBoard"}, "@src ws", {"id": 2, "op": "getBoard"})
        self.assertTrue(board[0]["stream_board"])
        self.assertError(board[2], "unsupported_transport")
        self.assertEqual(out[1]["result"]["subscribed"], True)
        self.assertError(out[3], "unsupported_transport")
        self.assertError(out[4], "unsupported_transport")
        self.assertTrue(out[5]["ok"])
        self.assertError(out[7], "unsupported_transport")
        self.assertTrue(out[8]["result"]["serial_subscribed"])


if __name__ == "__main__":
    unittest.main()
