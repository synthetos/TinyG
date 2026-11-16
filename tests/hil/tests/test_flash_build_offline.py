"""Build selection must return the newly built image, never a stale normal HEX."""
import pytest
from tinyg_hil import flash


@pytest.mark.parametrize('make_args, directory', [
    ((), 'build'),
    (('TEST_HOOKS=1',), 'build-testhooks'),
    (('TEST_HOOKS=0',), 'build-testhooks'),
    (('FW_BUILD=449.99', 'TEST_HOOKS=1'), 'build-fb449.99'),
    (('MOTION_PROFILE=8',), 'build-profile8'),
    (('MOTION_PROFILE=8', 'FW_BUILD=449.99'), 'build-profile8'),
    (('FW_BUILD=449.99', 'MOTION_PROFILE=8'), 'build-profile8'),
    (('MOTION_PROFILE=2', 'MOTION_PROFILE=8'), 'build-profile8'),
    (('MOTION_PROFILE=8', 'MOTION_PROFILE='), 'build'),
])
@pytest.mark.parametrize('firmware_directory', [False, True])
def test_build_returns_selected_image(tmp_path, monkeypatch, make_args, directory, firmware_directory):
    src = tmp_path / 'firmware' / 'tinyg'
    (src / 'build').mkdir(parents=True)
    (src / 'build' / 'tinyg.hex').write_text('stale normal image')
    calls = []

    def make(argv, **kwargs):
        calls.append(argv)
        target = src / directory / 'tinyg.hex'
        target.parent.mkdir(exist_ok=True)
        target.write_text('new selected image')

    monkeypatch.setattr(flash.subprocess, 'run', make)
    monkeypatch.setattr(flash, 'toolchain_id', lambda: ('host:test', None))
    result = flash.build(src if firmware_directory else tmp_path, make_args)
    assert calls == [['make', '-C', str(src.resolve()), '-j8', '-B', *make_args]]   # no stamp yet: full build
    assert (src / directory / '.toolchain').read_text().strip() == 'host:test'
    assert result == src / directory / 'tinyg.hex'
    assert result.read_text() == 'new selected image'
