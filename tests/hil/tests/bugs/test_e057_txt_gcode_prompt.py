"""E057: {"txt":"<gcode>"} prints a text-mode prompt into the JSON stream and reports status 0.

controller.c:339-342 runs txt G-code through text_response(gc_gcode_parser(str), ...), which
prints 'tinyg [mm] ok>' regardless of comm mode (text_parser.c:170-187). The dispatcher returns
void and the kernal returns STAT_OK (json_parser.c:163), so a G-code error only shows in the prompt.
"""
import pytest

pytestmark = pytest.mark.dialect("d449")     # 440 has no txt container


@pytest.mark.bug("E057", on=["d449"])
def test_txt_gcode_emits_only_json(tg):
    resp = tg.send_json({"txt": "G90"}, check=True)
    stray = [e.raw for e in resp.events if e.kind in ("prompt", "text", "malformed")]
    assert stray == []


@pytest.mark.bug("E057", on=["d449"])
def test_txt_gcode_error_in_footer(tg):
    assert tg.send_json({"txt": "M200"}).status == tg.dialect.status("MCODE_COMMAND_UNSUPPORTED")
