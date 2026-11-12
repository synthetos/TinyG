"""tinyg_hil.mso without hardware: a stand-in worker replays a canned result (offline)."""
import json
import os
import sys

import pytest

from tinyg_hil import mso

BENCH = {"serial": "TEST", "digital": {"p0c0": "M1 step", "p0c1": "M1 dir", "p2c3": "M4 step"},
         "analog": {"0": "spindle PWM"}}

FAKE = r'''
import json, sys
req = json.loads(sys.argv[1])
assert req["digital"] == [[0, 0], [2, 3]] and req["analog"] == [0] and req["pwm_window"] == 0.5 and req["pwm_threshold"] == 1.5, req
print("ARMED 1000.5", flush=True)
print("STARTED 1008.25", flush=True)
rows = [{"t": 0.0, "duty": 0.2, "freq_hz": 100.0}, {"t": 0.5, "duty": 0.2, "freq_hz": 100.0}]
json.dump({"rate": req["rate"], "initial": {"p0c0": 0, "p2c3": 1},
           "transitions": {"p0c0": [[0.1, 1], [0.2, 0]], "p2c3": [[0.3, 0]]},
           "analog": {"a0": {"npy": "unused.npy", "n": 0, "min": 0.0, "max": 3.0, "pwm": rows}}}, open(req["out"], "w"))
'''


@pytest.fixture
def fake_worker(tmp_path, monkeypatch):
    script = tmp_path / "fake_worker.py"
    script.write_text(FAKE)
    monkeypatch.setenv("TINYG_MSO_WORKER_CMD", f"{sys.executable} {script}")
    return script


def test_capture_maps_bench_names_and_reads_edges(fake_worker, tmp_path):
    with mso.Capture(BENCH, digital=["M1 step", "M4 step"], analog=["spindle PWM"], seconds=1, rate=100_000,
                     save_as=tmp_path / "cap") as cap:
        assert cap.armed_wall == 1000.5 and cap.started_wall == 1008.25
    assert cap.edges() == {"M1 step": [(0.1, 1), (0.2, 0)], "M4 step": [(0.3, 0)]}
    assert cap.initial() == {"M1 step": 0, "M4 step": 1}
    rows = cap.pwm("spindle PWM")
    for name in ("spindle PWM", "a0", "0", 0):
        assert cap.pwm(name) == rows
        assert cap.analog_path(name) == "unused.npy"
    assert all(abs(r["freq_hz"] - 100) < 0.5 and abs(r["duty"] - 0.2) < 0.01 for r in rows)


def test_unknown_channel_is_refused():
    with pytest.raises(mso.MsoError):
        mso.Capture(BENCH, digital=["M9 step"])


def test_exclusive_access_names_the_fix(tmp_path, monkeypatch):
    script = tmp_path / "busy.py"
    script.write_text('print("USBDeviceOpen failed: exclusive access and device already open"); raise SystemExit(1)\n')
    monkeypatch.setenv("TINYG_MSO_WORKER_CMD", f"{sys.executable} {script}")
    with pytest.raises(mso.MsoError, match="SALEAE_IGNORE_DEVICE"):
        with mso.Capture(BENCH, digital=["M1 step"], seconds=1):
            pass


def test_pwm_windows_on_a_clean_square_wave():
    np = pytest.importorskip("numpy")               # the worker's helper; the harness itself has no numpy
    rate = 200_000
    t = np.arange(rate) / rate
    y = ((t * 250) % 1.0 < 0.125).astype(np.float32) * 3.3
    rows = mso.pwm_windows(y, rate, window=0.25)
    assert len(rows) == 4 and all(abs(r["freq_hz"] - 250) < 1 and abs(r["duty"] - 0.125) < 0.005 for r in rows)


def test_pwm_windows_keeps_a_noisy_steady_line_steady():
    np = pytest.importorskip("numpy")
    y = (np.random.default_rng(1).normal(0.0, 0.05, 100_000)).astype(np.float32)    # 50 mV of noise around 0 V
    rows = mso.pwm_windows(y, 100_000, window=0.25)
    assert all(r["duty"] == 0.0 and r["freq_hz"] == 0.0 for r in rows)


def test_worker_that_never_starts_recording_fails(tmp_path, monkeypatch):
    script = tmp_path / "no_start.py"
    script.write_text('print("ARMED 1.0", flush=True)\nraise SystemExit(1)\n')
    monkeypatch.setenv("TINYG_MSO_WORKER_CMD", f"{sys.executable} {script}")
    with pytest.raises(mso.MsoError, match="did not start recording"):
        with mso.Capture(BENCH, digital=["M1 step"], seconds=1):
            pass


def test_worker_reads_the_start_time_from_the_device_log(capsys):
    import datetime
    import threading
    import types

    from tinyg_hil import mso_worker
    log = ("[2026-11-04 20:36:40.102133] [I] [tid 1] [main] [data_recorder.cpp:236] Waiting for instruments...\n"
           "[2026-11-04 20:36:43.408633] [I] [tid 1] [main] [capture_commands.cpp:50] recording started. waiting\n")
    mso_worker.watch_start(types.SimpleNamespace(last_stdout=log), threading.Event())
    want = datetime.datetime(2026, 11, 4, 20, 36, 43, 408633).timestamp()
    assert capsys.readouterr().out == f"STARTED {want!r}\n"


@pytest.mark.parametrize('stubborn',[False,True])
def test_failed_persistent_capture_releases_recorder_before_next_capture(tmp_path, monkeypatch, stubborn):
    import fcntl
    import textwrap
    lock=tmp_path/'recorder.lock'
    script=tmp_path/'server.py'
    child="import fcntl,signal,sys,time; f=open(sys.argv[1],'w'); fcntl.flock(f,fcntl.LOCK_EX); " + ("signal.signal(signal.SIGTERM,signal.SIG_IGN); " if stubborn else "") + "print('locked',flush=True); time.sleep(500)"
    script.write_text(textwrap.dedent('''
        import json,sys,subprocess,signal
        print('READY',flush=True)
        for line in sys.stdin:
            r=json.loads(line);i=r['id']
            if i=='warmup':print('DONE warmup',flush=True);continue
            child=subprocess.Popen([sys.executable,'-c',CHILD,LOCK],stdout=subprocess.PIPE,text=True)
            assert child.stdout.readline().strip()=='locked'
            if STUBBORN:signal.signal(signal.SIGTERM,signal.SIG_IGN)
            print('ARMED '+i+' 1',flush=True);print('STARTED '+i+' 2',flush=True)
            child.wait()
    ''').replace('LOCK',repr(str(lock))).replace('STUBBORN',repr(stubborn)).replace('CHILD',repr(child)))
    monkeypatch.setattr(mso,'_persistent',lambda:True)
    monkeypatch.setattr(mso,'worker_command',lambda wheel,req:[sys.executable,str(script),req])
    monkeypatch.setattr(mso,'ABORT_TIMEOUT',.1,raising=False)
    monkeypatch.setattr(mso._Server,'proc',None)
    proc=None
    try:
        with pytest.raises(RuntimeError,match='stream refused'):
            with mso.Capture(BENCH,digital=['M1 step'],seconds=500,save_as=tmp_path/'cap') as cap:
                proc=cap.server
                raise RuntimeError('stream refused')
        assert proc.poll() is not None
        assert mso._Server.proc is None
        with lock.open('a') as f:
            fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB)
        # A subsequent capture can arm immediately; abort it too without waiting 500 s.
        with pytest.raises(RuntimeError,match='next stream'):
            with mso.Capture(BENCH,digital=['M1 step'],seconds=500,save_as=tmp_path/'next'):
                raise RuntimeError('next stream')
    finally:
        # Test-first cleanup touches only this test's descendants, never instruments.
        if proc and proc.poll() is None:
            import subprocess as sp
            children=sp.check_output(['pgrep','-P',str(proc.pid)],text=True).split()
            for pid in children:
                try:os.kill(int(pid),9)
                except ProcessLookupError:pass
            proc.kill();proc.wait()
        mso._Server.proc=None
