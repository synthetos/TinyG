"""flash.make_command: host make by default, the pinned CI toolchain container with TINYG_BUILD_IMAGE."""
import os
import pathlib

from tinyg_hil import flash


def test_host_make_without_image(monkeypatch, tmp_path):
    monkeypatch.delenv("TINYG_BUILD_IMAGE", raising=False)
    assert flash.make_command(tmp_path, ["TEST_HOOKS=1"]) == ["make", "-C", str(tmp_path.resolve()), "-j8",
                                                              "TEST_HOOKS=1"]


def test_container_make_with_image(monkeypatch, tmp_path):
    monkeypatch.setenv("TINYG_BUILD_IMAGE", "tinyg-avr-ci")
    cmd = flash.make_command(tmp_path, ["TEST_HOOKS=1"])
    src = str(pathlib.Path(tmp_path).resolve())
    assert cmd[:3] == ["docker", "run", "--rm"]
    assert f"{src}:{src}" in cmd and cmd[cmd.index("-w") + 1] == src     # mounted at its own path
    assert cmd[cmd.index("--user") + 1] == f"{os.getuid()}:{os.getgid()}"  # build files owned by the host user
    assert cmd[-4:] == ["tinyg-avr-ci", "make", "-j8", "TEST_HOOKS=1"]


def test_blank_image_means_host(monkeypatch, tmp_path):
    monkeypatch.setenv("TINYG_BUILD_IMAGE", "  ")
    assert flash.make_command(tmp_path)[0] == "make"
