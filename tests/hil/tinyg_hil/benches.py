"""Bench wiring: the Atmel-ICE, Saleae and analyzer channels that belong to each serial port.

The tracked config/benches.json is an example with placeholder serials. The real file for a machine
lives outside the repository: $TINYG_BENCHES if set, else ~/.config/tinyg-hil/benches.json. The
example is used only when neither exists.
"""
import json
import os
import pathlib

EXAMPLE = pathlib.Path(__file__).resolve().parent.parent / "config" / "benches.json"


def path() -> pathlib.Path:
    env = os.environ.get("TINYG_BENCHES")
    if env:
        return pathlib.Path(env).expanduser()
    local = pathlib.Path.home() / ".config" / "tinyg-hil" / "benches.json"
    return local if local.exists() else EXAMPLE


def load() -> dict:
    return json.loads(path().read_text())


def bench(port) -> dict:
    return load().get(port) or {}
