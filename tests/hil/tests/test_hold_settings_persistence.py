"""Removed output-pause controls must not silently accept writes."""
import pytest

@pytest.mark.dialect('d449')
@pytest.mark.parametrize('token', ['spph', 'spde', 'coph'])
def test_removed_output_pause_setting(tg, token):
    assert tg.send_json({token: None}).status != 0
    assert tg.send_json({token: 1}).status != 0
