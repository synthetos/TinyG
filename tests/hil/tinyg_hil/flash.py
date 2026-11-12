"""Build firmware and flash it through the xboot bootloader."""
import datetime
import hashlib
import json
import os
import uuid
import pathlib
import re
import subprocess
import time

from .driver import NoResponse, TinyG
from .state import flash_record, write_flash_record, state_dir


def build(source_dir, make_args=()) -> pathlib.Path:
    """Run make in firmware/tinyg of a checkout; returns the hex path.

    make_args e.g. ["TEST_HOOKS=1"] (the Makefile then builds into build-testhooks/).
    """
    src = pathlib.Path(source_dir)
    if (src / "firmware" / "tinyg").is_dir():
        src = src / "firmware" / "tinyg"
    subprocess.run(["make", "-C", str(src), "-j8", *make_args], check=True, capture_output=True)
    builddir = "build-testhooks" if any(a.startswith("TEST_HOOKS=") for a in make_args) else "build"
    for a in make_args:
        if a.startswith("FW_BUILD="):
            builddir = "build-fb" + a.split("=", 1)[1]
    return src / builddir / "tinyg.hex"


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
        "dirty": dirty, "stock": bool(stock and not dirty and hex_path.parent.name == "build"), "stock_ref": stock_ref,
        "build_dir": hex_path.parent.name, "test_hooks": hex_path.parent.name == "build-testhooks",
    }


def read_provenance(port) -> dict:
    """The flash record for a port ({} if none). Older records held just the ELF path."""
    record = flash_record(port)
    if not record.exists():
        return {}
    text = record.read_text().strip()
    try:
        return json.loads(text)
    except ValueError:
        return {"elf": text, "legacy": True} if text else {}


def _program(cmd, timeout):
    """Durable start record before programming; keep complete output on every exit."""
    directory = state_dir() / 'flash-attempts'
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / (uuid.uuid4().hex + '.json')
    entry = dict(started_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                 command=cmd, timeout=timeout, state='started')

    def save():
        # Replace atomically so a killed process leaves at least its start record.
        pending = path.with_suffix('.pending')
        with pending.open('w', encoding='utf-8') as stream:
            json.dump(entry, stream, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        pending.replace(path)

    def output(value):
        return value.decode(errors='replace') if isinstance(value, bytes) else (value or '')

    save()  # If the log cannot be saved, do not begin programming.
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except BaseException as error:
        entry.update(state='interrupted', error=repr(error),
                     stdout=output(getattr(error, 'stdout', None)),
                     stderr=output(getattr(error, 'stderr', None)))
        try:
            save()
        except Exception as log_error:
            error.add_note(f'flash log update failed: {log_error}')
        error.add_note(f'avrdude attempt log: {path}')
        raise
    entry.update(state='finished', returncode=proc.returncode,
                 stdout=output(getattr(proc, 'stdout', None)), stderr=output(proc.stderr))
    save()
    if proc.returncode != 0:
        raise RuntimeError(f'avrdude failed ({proc.returncode}); log: {path}:\n{proc.stderr[-2000:]}')
    return proc


def flash(tg: TinyG, hex_path, *, expect_fb: float | None = None, avrdude: str = "avrdude",
          timeout: float = 180.0) -> dict:
    """Enter xboot with {"boot":1}, program with avrdude, reconnect. Returns the new identity.

    The hex is built with -R .eeprom, so settings are only lost if the new build changes fb
    (config_init then loads defaults). Take a snapshot first.
    """
    port = tg.transport.port
    # A prior interrupted flash may have left the serial port closed. Open
    # without probing: the board may still be in xboot, not application firmware.
    if not tg.transport.is_open:
        tg.transport.open()
    tg.transport.write(b'{"boot":1}\n')     # the reply may be cut off by the reset
    try:
        time.sleep(0.3)
        tg.close()
        time.sleep(0.5)
        cmd = [avrdude, "-p", "x192a3", "-c", "avr109", "-b", "115200", "-P", port,
               "-U", f"flash:w:{hex_path}:i"]
        _program(cmd, timeout)
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
        write_flash_record(port, flash_provenance(hex_path))    # read by record.py and ice.py
        if expect_fb is not None and abs(identity["fb"] - expect_fb) > 0.001:
            raise RuntimeError(f"flashed board reports fb {identity['fb']}, expected {expect_fb}")
        return identity
    except BaseException as error:
        # Leave a usable transport for recovery/teardown even when avrdude or
        # pytest-timeout interrupts us. Never hide the original flash failure.
        if not tg.transport.is_open:
            try:
                tg.transport.open()
            except Exception as reopen_error:
                error.add_note(f"serial reopen after failed flash: {reopen_error}")
        raise
