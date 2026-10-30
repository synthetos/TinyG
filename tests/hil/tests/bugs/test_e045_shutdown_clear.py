"""E045: SHUTDOWN can never be cleared; only ^X recovers.

controller.c:152 dispatches _shutdown_handler() ahead of _dispatch_control() (:157), and
controller.c:358-366 returns STAT_EAGAIN whenever the machine is in MACHINE_SHUTDOWN, so {clear:n}
(promised at canonical_machine.c:796-797, handled by cm_clear at :689-690) and every other command
go unanswered.
"""
import pytest

pytestmark = pytest.mark.dialect("d449")


@pytest.mark.motion     # SHUTDOWN blinks the indicator LED, which is the spindle DIR output (E019)
@pytest.mark.resets_board
@pytest.mark.bug("E045", on=["d449"])
def test_clear_exits_shutdown(tg):
    tg.send_json({"shutd": None})
    try:
        tg.send_json({"clear": None}, timeout=3)
        assert tg.stat() == tg.dialect.stat("READY")
    finally:
        tg.reset()


def test_clear_exits_alarm(tg):
    tg.send_json({"alarm": None})
    assert tg.stat() == tg.dialect.stat("ALARM")
    tg.send_json({"clear": None}, check=True)
    assert tg.stat() == tg.dialect.stat("PROGRAM_STOP")
