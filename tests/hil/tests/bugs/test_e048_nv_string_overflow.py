"""E048: nv_copy_string() lets a string of exactly the free length through without room for its NUL.

config.c:791 checks `nvStr.wp + strlen(src) > NV_SHARED_STRING_LEN` (256), so a string that exactly
fills the rest of the shared pool passes and strcpy writes its terminator into nvStr.magic_end; the
next assertion check PANICs (90).

In JSON mode the raw G-code line is copied into the pool for the echo (controller.c), a (MSG ...)
comment adds its text, and the response footer is copied last, so for some comment length the footer
lands exactly on the end of the pool. Lines stay under the 199-character line-mode limit; the sweep
covers the boundary whatever the footer's exact length.
"""
import pytest

pytestmark = pytest.mark.dialect("d449")


@pytest.mark.bug("E048", on=[])
def test_shared_string_pool_boundary_does_not_panic(tg):
    tg.send_json({"fb": None}, check=True)              # JSON mode: raw lines are echoed through nv
    try:
        for k in range(95, 150):
            line = "G4 P0 (MSG " + "x" * k + ")"
            tg.send_line(line, timeout=5)
            assert tg.stat() != tg.dialect.stat("PANIC"), f"PANIC at comment length {k}"
    finally:
        if tg.stat() == tg.dialect.stat("PANIC"):
            tg.reset()
