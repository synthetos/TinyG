"""Text command names must match in full; inspect the real text prompt.

The JSON txt container always returns status OK, even when its nested text
parser rejects a name. JSON long keys are independently length-checked before
copying into the token buffer.
"""
import pytest

pytestmark = pytest.mark.dialect("d449")


@pytest.mark.bug("N17", on=[])
def test_text_name_suffix_rejected(tg, record):
    lines = tg.send_text("$clearanything")
    record("text_prompt", lines)
    assert "err: Unrecognized command or config name" in lines[-1], lines


def test_json_long_name_rejected_before_token_copy(tg, record):
    # The parser cannot echo an overlong key; inspect the raw error footer.
    response = tg.send_json({"clearanything": None}, correlate=False)
    record("json_long_name", {"status": response.status, "body": response.body})
    assert response.status == tg.dialect.status("INPUT_EXCEEDS_MAX_LENGTH")
