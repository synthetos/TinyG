"""E024: readable feed/spindle factors and explicit unsupported extensions.

Feed permission is implemented by M48/M49/M50. Spindle remains unity;
its unsupported M51 and dotted extensions must never silently succeed.
"""
import pytest


@pytest.mark.bug("E024", on=["d440"])
@pytest.mark.parametrize("token", ["sso", "mfo"])
def test_override_token(tg, token):
    resp = tg.send_json({token: None})
    assert resp.status == 0
    assert isinstance(resp.body.get(token), (int, float))


@pytest.mark.bug("E024", on=["d440"])
@pytest.mark.parametrize("line", ["M50.1 P0.5",
                                  "M50.2 P0.5", "M50.3 P0.5", "M51", "M51.1 P0.5"])
def test_unimplemented_override_is_explicit(tg, line):
    assert tg.send_gcode(line).status == 132  # unsupported M code, not a silent success
