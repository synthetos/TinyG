"""Opt-in upgrade from a supplied 450.03/450.04 source, without restoring between flashes.

TINYG_LAYOUT2_SOURCE selects the old tree. The bench operator selects each source
in turn; the 450.04 case saves output pause enabled before the upgrade. Existing
N92 ICE coverage exercises master upgrades and power cuts separately.
"""
import os
from pathlib import Path
import pytest
from tinyg_hil import flash as flasher, snapshot
from tinyg_hil.state import state_dir

SOURCE = Path(__file__).resolve().parents[4]
REMOVED = {'spph', 'spde', 'coph', '_hld'}
pytestmark = [pytest.mark.destructive, pytest.mark.resets_board,
              pytest.mark.persists, pytest.mark.dialect('d449'), pytest.mark.timeout(900)]


def test_layout2_upgrade_preserves_settings_and_report_fields(tg, record):
    path = os.environ.get('TINYG_LAYOUT2_SOURCE')
    if not path:
        pytest.skip('set TINYG_LAYOUT2_SOURCE to the 450.03 or 450.04 source')
    old = Path(path).resolve()
    old_hex, new_hex = flasher.build(old), flasher.build(SOURCE)
    saved = snapshot.take(tg)
    snapshot.save(saved, state_dir() / 'layout3-upgrade')
    upgraded = False
    failure = None
    try:
        flasher.flash(tg, old_hex, expect_fb=flasher.source_build(old))
        restored = snapshot.restore(tg, saved)
        assert not restored['failed'], restored
        tg.set('1sa', .9)
        tg.set('prbp', 1)
        pause = tg.send_json({'spph': 1})
        assert pause.status in (0, tg.dialect.status('FUNCTION_IS_STUBBED'))
        # Prefix and shifted suffix entries must both survive by name.
        snapshot.set_status_report(tg, ['stat', 'posx', 'spe', 'sps', 'cof', 'com'])
        before = snapshot.take(tg)
        before['values']['prbp'] = tg.get('prbp')
        flasher.flash(tg, new_hex, expect_fb=flasher.source_build(SOURCE))
        upgraded = True
        for reset in (False, True):
            if reset:
                tg.reset()
            mismatches = {}
            for token, want in before['values'].items():
                if token in REMOVED or token in snapshot.IDENTITY or snapshot.INTERNAL.match(token):
                    continue
                got = tg.get(token)
                if not tg.values_equal(token, got, want):
                    mismatches[token] = {'before': want, 'after': got}
            assert not mismatches, mismatches
            fields = tg.get('sr')
            assert list(fields) == before['sr']
            for token in REMOVED - {'_hld'}:
                assert tg.send_json({token: None}).status != 0
        record('layout3_upgrade', {'source': str(old), 'before': before, 'verified': True})
    except BaseException as error:
        failure = error
        raise
    finally:
        try:
            if not upgraded:
                flasher.flash(tg, new_hex, expect_fb=flasher.source_build(SOURCE))
            clean = dict(saved, values={k:v for k,v in saved['values'].items() if k not in REMOVED})
            restored = snapshot.restore(tg, clean)
            assert not restored['failed'], restored
        except BaseException as recovery_error:
            if failure is None:
                raise
            failure.add_note(f"upgrade recovery also failed: {recovery_error}")
            record('layout3_recovery_error', str(recovery_error))
