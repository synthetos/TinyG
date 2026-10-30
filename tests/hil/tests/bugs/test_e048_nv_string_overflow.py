"""E048: nv_copy_string() lets a string of exactly the free length through without room for its NUL.

config.c:791 checks `nvStr.wp + strlen(src) > NV_SHARED_STRING_LEN` (256), so a 256-character string
passes and strcpy writes its terminator into nvStr.magic_end; the next assertion check PANICs (90).
controller.c:307 copies every raw G-code line there (JSON-mode echo) and ignores the return value.
Line mode caps lines at 199 characters, so the reproduction uses character mode ($rxm=0).
"""
import pytest

pytestmark = [pytest.mark.persists, pytest.mark.dialect("d449")]


@pytest.mark.bug("E048", on=["d449"])
def test_256_char_gcode_line_does_not_panic(tg, keep):
    keep("rxm")
    tg.set("rxm", 0)                               # character mode: no 199-character line limit
    tg.send_json({"sr": None})                     # JSON mode, so the raw line is echoed through nv
    line = "G4 P0 (" + "x" * (256 - 8) + ")"
    assert len(line) == 256
    try:
        tg.send_line(line, timeout=5)
        tg.drain(quiet=0.5)
        assert tg.stat() != tg.dialect.stat("PANIC")
    finally:
        if tg.stat() == tg.dialect.stat("PANIC"):
            tg.reset()
