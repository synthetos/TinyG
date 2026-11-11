"""No serial ports: lock changeover and flash record provenance."""
import fcntl
import json
import os
import subprocess
import sys
from pathlib import Path
import pytest
from tinyg_hil import state
from tinyg_hil.lock import BoardBusy, BoardLock
from tinyg_hil.flash import read_provenance

@pytest.fixture
def paths(tmp_path, monkeypatch):
    monkeypatch.setenv('TINYG_HIL_STATE', str(tmp_path / 'new'))
    legacy = tmp_path / 'legacy'; legacy.mkdir()
    monkeypatch.setattr(state, 'LEGACY_STATE', legacy)
    return state.port_path('offline-board', 'lock'), state.port_path('offline-board', 'lock', legacy=True)

def try_other_process(path):
    return subprocess.run([sys.executable, '-c', '''import fcntl,sys
with open(sys.argv[1], 'a+') as f:
    try: fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError: sys.exit(7)
''', str(path)], check=False).returncode

def test_new_holder_excludes_old_and_new(paths):
    with BoardLock('offline-board').acquire('offline'):
        for p in paths: assert try_other_process(p) == 7
        with pytest.raises(BoardBusy): BoardLock('offline-board').acquire('second')
    for p in paths: assert try_other_process(p) == 0

def test_old_holder_excludes_new(paths):
    with paths[1].open('a+') as old:
        fcntl.flock(old, fcntl.LOCK_EX | fcntl.LOCK_NB)
        lock = BoardLock('offline-board')
        with pytest.raises(BoardBusy): lock.acquire('new')
        assert lock.fd is None
    with lock.acquire('after old releases'): assert try_other_process(paths[1]) == 7

def test_second_lock_failure_releases_first(paths):
    paths[0].parent.mkdir()
    with paths[0].open('a+') as held:
        fcntl.flock(held, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(BoardBusy): BoardLock('offline-board').acquire('new')
        assert try_other_process(paths[1]) == 0

def test_record_fallback_and_old_worktree_reflash(paths):
    new = state.port_path('offline-board', 'flashed')
    old = state.port_path('offline-board', 'flashed', legacy=True)
    assert read_provenance('offline-board') == {}
    old.write_text('/old/image.elf')
    assert read_provenance('offline-board') == {'elf': '/old/image.elf', 'legacy': True}
    new.parent.mkdir(exist_ok=True); new.write_text(json.dumps({'hex': 'new'}))
    os.utime(old, ns=(100, 100)); os.utime(new, ns=(200, 200))
    assert read_provenance('offline-board') == {'hex': 'new'}
    old.write_text(json.dumps({'hex': 'reflashed-by-old-worktree'}))
    assert read_provenance('offline-board')['hex'] == 'reflashed-by-old-worktree'

def test_night_reader_uses_same_selection(paths):
    new = state.port_path('offline-board', 'flashed'); new.parent.mkdir(exist_ok=True)
    new.write_text('current-image')
    script = Path(__file__).resolve().parents[1] / 'tools/overnight/night-lib.sh'
    result = subprocess.run(['zsh', '-c', 'source "$1"; P=offline-board; _flash_record', '_', str(script)],
                            text=True, capture_output=True, check=True)
    assert result.stdout.strip() == str(new)


def test_flash_write_is_persistent_and_leaves_legacy_alone(paths):
    old = state.port_path('offline-board', 'flashed', legacy=True)
    old.write_text('old-image')
    state.write_flash_record('offline-board', {'hex': 'new-image'})
    assert read_provenance('offline-board') == {'hex': 'new-image'}
    assert old.read_text() == 'old-image'
    assert list(state.state_dir().glob('*.flashed.*')) == []


def test_new_holders_with_different_state_dirs_still_exclude(paths, monkeypatch, tmp_path):
    with BoardLock('offline-board').acquire('first directory'):
        monkeypatch.setenv('TINYG_HIL_STATE', str(tmp_path / 'other'))
        with pytest.raises(BoardBusy): BoardLock('offline-board').acquire('second directory')


def test_default_state_is_persistent(monkeypatch):
    monkeypatch.delenv('TINYG_HIL_STATE', raising=False)
    assert state.state_dir() == Path('/Volumes/External/projects/TinyG-work/state')
