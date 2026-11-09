"""E151: {"sys":n} drops os and mangles fstk/adc3; text $sys prints bare numbers.

get_grp() puts one nvObj per sys row in the body: 36 children plus the parent overran the 36-object
body into the footer slot, so os was overwritten. fstk, adc3 and os lacked F_NOSTRIP, so the group
listing cut strlen("sys") characters off their tokens ("k", "3"). spep, spdp, prbp and os printed with
the generic tx_print_int, i.e. a bare value in text mode.
"""
import pytest

pytestmark = pytest.mark.dialect("d449")

EXPECTED = ["fb", "fv", "hp", "hv", "id", "ja", "ct", "sl", "st", "mt", "ej", "jv", "js", "tv", "qv", "sv",
            "si", "ec", "ee", "ex", "rxm", "baud", "gpl", "gun", "gco", "gpa", "gdi", "spep", "spdp", "spph", "spde", "coph", "prbp", "os"]


def test_sys_group_is_whole(tg):
    tg.transport.write(b'{"sys":n}\n')
    ev = tg.wait_for(lambda e: e.kind == "response" and isinstance(e.body, dict) and "sys" in e.body, 5.0)
    keys = list(ev.body["sys"])
    assert keys == EXPECTED, keys
    assert ev.data["f"][1] == 0
    for token in ("fstk", "adc3", "os"):                 # still readable on their own
        assert tg.get(token) is not None


@pytest.mark.persists
def test_text_sys_labels_every_setting(tg, keep):
    keep("ej")
    tg.set("ej", 0)
    tg.transport.write(b"$sys\n")
    lines = [e.raw for e in tg.drain(quiet=1.0)]
    text = "\n".join(lines)
    for token in ("spep", "spdp", "spph", "spde", "coph", "prbp", "os"):
        assert f"[{token}]" in text, f"$sys has no labelled {token} line: {lines[-8:]}"
    assert not any(l.strip().isdigit() for l in lines), f"bare values in $sys: {lines[-8:]}"
