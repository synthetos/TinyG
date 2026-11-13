"""Decimal spellings of the live feed request use the documented endpoints."""
import pytest

pytestmark = [pytest.mark.dialect('d449'), pytest.mark.resets_board]


@pytest.mark.parametrize('word,expected', [
    ('0.05', .05), ('0.050', .05), ('5e-2', .05), ('0.051', .051), ('2.0', 2.0),
])
def test_override_factor_number(tg, word, expected, record):
    try:
        tg.reset()
        reply = tg.send_line('{"rt":{"mfo":' + word + '}}')
        assert reply.status == 0, reply
        assert reply.body['rt']['mfo'] == pytest.approx(expected)
        read = tg.send_line('{"rt":{"mfo":null}}')
        assert read.status == 0, read
        assert read.body['rt']['mfo'] == pytest.approx(expected)
        record('factor_number', {'word': word, 'requested': read.body['rt']['mfo']})
    finally:
        tg.reset()


@pytest.mark.parametrize('word', ['0.049', '0.04999999', '2.001'])
def test_override_factor_outside_range(tg, word):
    try:
        tg.reset()
        rejected = tg.send_line('{"rt":{"mfo":' + word + '}}')
        assert rejected.status == 110
        assert rejected.body['rt']['mfo'] == pytest.approx(round(float(word), 3))
        read = tg.send_line('{"rt":{"mfo":null}}')
        assert read.status == 0
        assert read.body['rt']['mfo'] == 1
    finally:
        tg.reset()
