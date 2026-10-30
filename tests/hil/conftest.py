import os
import pathlib

import pytest

from tinyg_hil import Guard, TinyG, snapshot
from tinyg_hil.lock import BoardLock
from tinyg_hil.record import RunRecorder

HERE = pathlib.Path(__file__).parent
# Response format the tests are written against; applied for the session and restored afterwards.
BASELINE = {"js": 1, "jv": 4, "ee": 0, "ec": 0, "tv": 1, "sv": 1, "qv": 0}


def pytest_addoption(parser):
    g = parser.getgroup("tinyg")
    g.addoption("--port", default=os.environ.get("TINYG_PORT"),
                help="TinyG serial port, e.g. /dev/cu.usbserial-XXXX [TINYG_PORT]. Never guessed.")
    g.addoption("--baud", type=int, default=115200)
    g.addoption("--allow-motion", action="store_true", default=os.environ.get("TINYG_ALLOW_MOTION") == "1",
                help="allow tests that move axes or switch outputs [TINYG_ALLOW_MOTION=1]")
    g.addoption("--allow-destructive", action="store_true", help="allow flashing and $defa")
    g.addoption("--no-reset", action="store_true", help="skip tests that reset the board")
    g.addoption("--transcript", default=None, help="write a timestamped raw serial transcript here")
    g.addoption("--snapshot-dir", default=str(HERE / ".snapshots"))
    g.addoption("--runs-dir", default=str(HERE / ".runs"), help="where run records for the dashboard go")
    g.addoption("--no-record", action="store_true", help="do not write run records")
    g.addoption("--run-purpose", default=None, choices=["stock-baseline", "fix-verify", "regression", "adhoc"],
                help="recorded with the run (default: stock-baseline if the flashed firmware is stock edge, else fix-verify)")
    g.addoption("--ice", action="store_true", help="enable Atmel-ICE tests (peek/poke over PDI)")
    g.addoption("--elf", default=None, help="ELF of the flashed build for --ice (default: recorded at flash time)")
    g.addoption("--lock-wait", type=float, default=600.0,
                help="seconds to wait for another user of the board to finish [default 600]")


def pytest_configure(config):
    config._tinyg_recorder = None
    if not config.getoption("--no-record") and not config.getoption("--collect-only"):
        config._tinyg_recorder = RunRecorder(config.getoption("--runs-dir"), HERE)


_CONFIG = {}


@pytest.hookimpl(trylast=True)
def pytest_sessionstart(session):
    _CONFIG["config"] = session.config


def pytest_runtest_logreport(report):
    config = _CONFIG.get("config")
    if config is not None and config._tinyg_recorder:
        config._tinyg_recorder.report(report)


def pytest_unconfigure(config):
    if getattr(config, "_tinyg_recorder", None):
        config._tinyg_recorder.finish()


def pytest_collection_modifyitems(config, items):
    for item in items:
        if "tg" in getattr(item, "fixturenames", ()):
            item.add_marker(pytest.mark.hardware)
        if item.get_closest_marker("motion") and not config.getoption("--allow-motion"):
            item.add_marker(pytest.mark.skip(reason="moves axes; pass --allow-motion"))
        if item.get_closest_marker("destructive") and not config.getoption("--allow-destructive"):
            item.add_marker(pytest.mark.skip(reason="destructive; pass --allow-destructive"))
        if item.get_closest_marker("ice") and not config.getoption("--ice"):
            item.add_marker(pytest.mark.skip(reason="needs the Atmel-ICE; pass --ice"))
        if item.get_closest_marker("resets_board") and config.getoption("--no-reset"):
            item.add_marker(pytest.mark.skip(reason="--no-reset"))


@pytest.fixture(scope="session")
def tg(request):
    """The board, in the BASELINE response format, with all settings restored at the end."""
    config = request.config
    port = config.getoption("--port")
    if not port:
        pytest.skip("no TinyG: pass --port or set TINYG_PORT")
    lock = BoardLock(port).acquire(f"pytest {' '.join(config.invocation_params.args)[:200]}",
                                   wait=config.getoption("--lock-wait"))
    request.addfinalizer(lock.release)
    transcript_path = config.getoption("--transcript")
    transcript = open(transcript_path, "w") if transcript_path else None
    board = TinyG(port, config.getoption("--baud"), transcript=transcript,
                  guard=Guard(allow_motion=config.getoption("--allow-motion")))
    board.connect()
    if config._tinyg_recorder:
        config._tinyg_recorder.board(board, config.getoption("--run-purpose"))
    # Text mode reads work at any jv, so the user's own comm settings are captured before changing them.
    original = {t: board.get_text(t) for t in BASELINE}
    for token, value in BASELINE.items():
        board.set(token, value)
    snap = snapshot.take(board)
    snap["values"].update({t: v for t, v in original.items() if v is not None})
    board.snapshot = snap
    board.snapshot_path = snapshot.save(snap, config.getoption("--snapshot-dir"))
    yield board
    board.drain()
    recover(board)
    result = snapshot.restore(board, snap)
    board.close()
    if transcript:
        transcript.close()
    if result["failed"]:
        pytest.fail(f"settings restore failed: {result['failed']} (snapshot {board.snapshot_path})")


def recover(board):
    """Get the board back to idle, ready and unalarmed, whatever the last test left behind."""
    try:
        stat, qr = board.stat(), board.get("qr")
    except Exception:  # noqa: BLE001 - silent board: only a reset helps
        stat, qr = None, None
    if stat is None or stat in (board.dialect.stat("SHUTDOWN"), board.dialect.stat("PANIC")) \
            or qr != board.planner_buffers:
        board.reset()       # stale queue (E069), unrecoverable state, or no answer
    if board.stat() == board.dialect.stat("ALARM"):
        board.clear_alarm()


@pytest.fixture(scope="session")
def ice(request, tg):
    """Atmel-ICE debug session on the same board (the tg session already holds the board lock)."""
    from tinyg_hil.ice import Ice, flashed_elf
    elf = request.config.getoption("--elf") or flashed_elf(tg.transport.port)
    if not elf:
        pytest.skip("no ELF for the flashed build: pass --elf or flash through tinyg_hil")
    session = Ice(elf=elf).open()
    yield session
    session.close()


@pytest.fixture
def switches(tg, ice):
    """Software switches on the unwired inputs (needs --ice); everything released after the test."""
    from tinyg_hil.switches import SoftSwitches
    soft = SoftSwitches(ice, normally_closed=(tg.get("st") == 1))
    soft.release_all()
    yield soft
    soft.release_all()


@pytest.fixture
def dialect(tg):
    return tg.dialect


@pytest.fixture
def keep(tg):
    """keep("si", "jv") saves those settings now and writes them back after the test."""
    saved = {}

    def _keep(*tokens):
        for token in tokens:
            saved.setdefault(token, tg.get(token))
    yield _keep
    tg.drain(quiet=0.05)
    for token, value in saved.items():
        if not tg.values_equal(token, tg.get(token), value):
            tg.set(token, value, allow="restore after test")


@pytest.fixture(autouse=True)
def _per_test(request):
    """Apply dialect/bug markers, then put the board in a known state before each hardware test."""
    if "tg" not in request.fixturenames:
        yield
        return
    board = request.getfixturevalue("tg")
    marker = request.node.get_closest_marker("dialect")
    if marker and board.dialect.name not in marker.args:
        pytest.skip(f"only on {marker.args}; board is {board.dialect.name}")
    for bug in request.node.iter_markers("bug"):
        if board.dialect.name in bug.kwargs.get("on", ("d440", "d449")):
            request.node.add_marker(pytest.mark.xfail(strict=True, reason=f"known bug {bug.args[0]}"))
    board.drain(quiet=0.05)
    recover(board)
    board.send_gcode("M2", check=True)     # resets G92, coordinate system, plane, distance mode, M5, M9
    board.send_gcode("G21", check=True)    # M2 does not reset units
    board.drain(quiet=0.05)
    recorder = request.config._tinyg_recorder
    if recorder:
        recorder.begin(request.node.nodeid, board, [m.args[0] for m in request.node.iter_markers("bug")],
                       sorted({m.name for m in request.node.iter_markers()}))
    yield
    if recorder:
        recorder.end(request.node.nodeid, board)
