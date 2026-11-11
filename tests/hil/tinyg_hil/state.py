"""Persistent harness state, with legacy records supported during changeover."""
import os
import json
import tempfile
from pathlib import Path

DEFAULT_STATE = Path('/Volumes/External/projects/TinyG-work/state')
LEGACY_STATE = Path('/tmp')


def state_dir():
    return Path(os.environ.get('TINYG_HIL_STATE') or DEFAULT_STATE).expanduser()


def port_path(port, suffix, *, legacy=False):
    return (LEGACY_STATE if legacy else state_dir()) / f'tinyg-hil-{Path(port).name}.{suffix}'


def flash_record(port):
    """Prefer persistent state, except when an old worktree flashed more recently."""
    paths = [port_path(port, 'flashed'), port_path(port, 'flashed', legacy=True)]
    existing = [p for p in paths if p.exists()]
    return max(existing, key=lambda p: p.stat().st_mtime_ns) if existing else paths[0]


def write_flash_record(port, provenance):
    """Publish a complete record in persistent state; never write legacy flash state."""
    path = port_path(port, 'flashed')
    path.parent.mkdir(parents=True, exist_ok=True)
    name = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                         prefix=path.name + '.', delete=False) as f:
            name = f.name
            json.dump(provenance, f, indent=1)
            f.flush()
            os.fsync(f.fileno())
        os.replace(name, path)
    finally:
        if name is not None:
            Path(name).unlink(missing_ok=True)
