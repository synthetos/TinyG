"""Harness fixture checks on hardware."""


def test_keep_restores(tg, keep):
    keep("si")
    original = tg.get("si")
    tg.set("si", original + 50)
    assert tg.get("si") == original + 50


def test_keep_restored(tg):
    assert tg.get("si") == tg.snapshot["values"]["si"]
