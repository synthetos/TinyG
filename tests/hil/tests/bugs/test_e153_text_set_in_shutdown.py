"""E153: text-mode $ settings are accepted, and persisted, in SHUTDOWN (and PANIC).

text_parser() rejects a SET only when machine_state == MACHINE_ALARM; JSON sets go through
cm_is_alarmed(), which also answers 205 (SHUTDOWN) and 206 (PANIC). A shutdown is meant to reject
SET commands until it is cleared.
"""
import pytest

pytestmark = pytest.mark.dialect("d449")


@pytest.mark.persists
@pytest.mark.resets_board
def test_text_set_rejected_in_shutdown(tg, keep):
    keep("sv")
    sv = tg.get("sv")
    other = 2 if sv != 2 else 1
    rejected = tg.dialect.status("COMMAND_REJECTED_BY_SHUTDOWN")
    tg.send_json({"shutd": None})
    try:
        assert tg.stat() == tg.dialect.stat("SHUTDOWN")
        assert tg.send_json({"sv": other}).status == rejected          # JSON already rejects it
        tg.transport.write(f"$sv={other}\n".encode())                 # the text path must reject it too
        tg.drain(quiet=0.5)
        assert tg.get("sv") == sv, "a text set changed sv during shutdown"
    finally:
        tg.reset()                                                   # leave no shutdown behind
