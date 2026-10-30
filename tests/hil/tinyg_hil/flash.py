"""Build firmware and flash it through the xboot bootloader."""
import datetime
import hashlib
import json
import pathlib
import re
import subprocess
import time

from .driver import NoResponse, TinyG


def build(source_dir) -> pathlib.Path:
    """Run make in firmware/tinyg of a checkout; returns the hex path."""
    src = pathlib.Path(source_dir)
    if (src / "firmware" / "tinyg").is_dir():
        src = src / "firmware" / "tinyg"
    subprocess.run(["make", "-C", str(src), "-j8"], check=True, capture_output=True)
    return src / "build" / "tinyg.hex"


def source_build(source_dir) -> float:
    """TINYG_FIRMWARE_BUILD from a checkout's tinyg.h."""
    src = pathlib.Path(source_dir)
    if (src / "firmware" / "tinyg").is_dir():
        src = src / "firmware" / "tinyg"
    text = (src / "tinyg.h").read_text(errors="replace")
    return float(re.search(r"#define\s+TINYG_FIRMWARE_BUILD\s+([\d.]+)", text).group(1))


def _git(cwd, *args):
    r = subprocess.run(["git", "-C", str(cwd), *args], capture_output=True, text=True)
    return r.stdout.strip() if r.returncode == 0 else ""


def flash_provenance(hex_path, stock_ref="origin/edge-0.97") -> dict:
    """Immutable description of the image being flashed, captured at flash time."""
    hex_path = pathlib.Path(hex_path).resolve()
    elf = hex_path.with_suffix(".elf")
    tree = hex_path.parent.parent if hex_path.parent.name == "build" else hex_path.parent   # firmware/tinyg
    commit = _git(tree, "rev-parse", "HEAD")
    stock = subprocess.run(["git", "-C", str(tree), "diff", "--quiet", stock_ref, "HEAD", "--", ".", ":!Makefile"],
                           capture_output=True).returncode == 0 if commit else False
    dirty = bool(_git(tree, "status", "--porcelain", "--", ".")) if commit else None
    return {
        "flashed_at": datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds"),
        "hex": str(hex_path), "hex_sha256": hashlib.sha256(hex_path.read_bytes()).hexdigest(),
        "elf": str(elf) if elf.exists() else "",
        "elf_sha256": hashlib.sha256(elf.read_bytes()).hexdigest() if elf.exists() else "",
        "commit": commit, "branch": _git(tree, "symbolic-ref", "--short", "-q", "HEAD"),
        "tree": _git(tree, "rev-parse", "HEAD:firmware/tinyg") if commit else "",
        "dirty": dirty, "stock": bool(stock and not dirty), "stock_ref": stock_ref,
    }


def read_provenance(port) -> dict:
    """The flash record for a port ({} if none). Older records held just the ELF path."""
    record = pathlib.Path("/tmp") / f"tinyg-hil-{pathlib.Path(port).name}.flashed"
    if not record.exists():
        return {}
    text = record.read_text().strip()
    try:
        return json.loads(text)
    except ValueError:
        return {"elf": text, "legacy": True} if text else {}


def flash(tg: TinyG, hex_path, *, expect_fb: float | None = None, avrdude: str = "avrdude",
          timeout: float = 180.0) -> dict:
    """Enter xboot with {"boot":1}, program with avrdude, reconnect. Returns the new identity.

    The hex is built with -R .eeprom, so settings are only lost if the new build changes fb
    (config_init then loads defaults). Take a snapshot first.
    """
    port = tg.transport.port
    tg.transport.write(b'{"boot":1}\n')     # the reply may be cut off by the reset
    time.sleep(0.3)
    tg.close()
    time.sleep(0.5)
    cmd = [avrdude, "-p", "x192a3", "-c", "avr109", "-b", "115200", "-P", port,
           "-U", f"flash:w:{hex_path}:i"]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    if proc.returncode != 0:
        raise RuntimeError(f"avrdude failed ({proc.returncode}):\n{proc.stderr[-2000:]}")
    # The first boot of a new fb writes every default to EEPROM before answering.
    deadline = time.monotonic() + 30
    while True:
        time.sleep(1.0)
        try:
            identity = tg.connect(timeout=3.0)
            break
        except NoResponse:
            if time.monotonic() > deadline:
                raise
    record = pathlib.Path("/tmp") / f"tinyg-hil-{pathlib.Path(port).name}.flashed"
    record.write_text(json.dumps(flash_provenance(hex_path), indent=1))    # read by record.py and ice.py
    if expect_fb is not None and abs(identity["fb"] - expect_fb) > 0.001:
        raise RuntimeError(f"flashed board reports fb {identity['fb']}, expected {expect_fb}")
    return identity
