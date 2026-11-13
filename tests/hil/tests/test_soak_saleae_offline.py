"""Optional soak captures must distinguish absence from capture faults."""
import urllib.error
import pytest
from tinyg_hil import saleae


def test_off_never_contacts_server(monkeypatch):
    monkeypatch.setattr(saleae, 'call', lambda *a, **kw: pytest.fail('unexpected RPC'))
    assert saleae.soak_capture_policy('off', True)[0] is False
    assert saleae.soak_capture_policy('auto', False)[0] is False


@pytest.mark.parametrize('error', [urllib.error.URLError('connection refused'), TimeoutError('timeout')])
def test_auto_records_unreachable_and_on_fails(monkeypatch, error):
    def call(*args, **kwargs): raise error
    monkeypatch.setattr(saleae, 'call', call)
    enabled, note = saleae.soak_capture_policy('auto', True)
    assert not enabled and 'unreachable' in note
    with pytest.raises(type(error)): saleae.soak_capture_policy('on', True)


@pytest.mark.parametrize('error', [RuntimeError('device failure'), urllib.error.HTTPError('url', 500, 'bad', {}, None)])
def test_auto_does_not_hide_server_errors(monkeypatch, error):
    def call(*args, **kwargs): raise error
    monkeypatch.setattr(saleae, 'call', call)
    with pytest.raises(type(error)): saleae.soak_capture_policy('auto', True)


def test_valid_modes_and_bounded_probe(monkeypatch):
    calls = []
    monkeypatch.setattr(saleae, 'call', lambda *a, **kw: calls.append((a, kw)))
    for mode in ('auto', 'on'):
        assert saleae.soak_capture_policy(mode, True)[0]
    assert all(k['timeout'] == 3 for _, k in calls)
    with pytest.raises(ValueError): saleae.soak_capture_policy('wrong', True)
    with pytest.raises(ValueError): saleae.soak_capture_policy('on', False)
