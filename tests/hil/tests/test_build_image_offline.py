"""flash.build toolchain selection: host make by default, the pinned CI toolchain container with TINYG_BUILD_IMAGE,
and a full rebuild whenever the build directory was not last built successfully by the same exact toolchain."""
import os
import pathlib
import subprocess

import pytest

from tinyg_hil import flash


def test_host_make_without_image(tmp_path):
    assert flash.make_command(tmp_path, ["TEST_HOOKS=1"]) == ["make", "-C", str(tmp_path.resolve()), "-j8",
                                                              "TEST_HOOKS=1"]


def test_container_make_uses_the_image_id(tmp_path):
    cmd = flash.make_command(tmp_path, ["TEST_HOOKS=1"], image_id="sha256:abc")
    src = str(pathlib.Path(tmp_path).resolve())
    assert cmd[:3] == ["docker", "run", "--rm"]
    assert f"{src}:{src}" in cmd and cmd[cmd.index("-w") + 1] == src     # mounted at its own path
    assert cmd[cmd.index("--user") + 1] == f"{os.getuid()}:{os.getgid()}"  # build files owned by the host user
    assert cmd[-4:] == ["sha256:abc", "make", "-j8", "TEST_HOOKS=1"]       # immutable ID, not the tag


def test_toolchain_id_resolves_image_and_host(monkeypatch):
    seen = []
    def check_output(argv, **kw):
        seen.append(argv)
        return "sha256:feed\n" if argv[0] == "docker" else "avr-gcc (GCC) 7.3.0\nCopyright\n"
    monkeypatch.setattr(flash.subprocess, "check_output", check_output)
    monkeypatch.setenv("TINYG_BUILD_IMAGE", "tinyg-avr-ci")
    assert flash.toolchain_id() == ("image:sha256:feed", "sha256:feed")
    assert seen[-1][:3] == ["docker", "image", "inspect"] and seen[-1][-1] == "tinyg-avr-ci"
    monkeypatch.setenv("TINYG_BUILD_IMAGE", "  ")
    stamp, image_id = flash.toolchain_id()
    assert image_id is None and stamp.startswith("host:") and stamp.endswith("avr-gcc (GCC) 7.3.0")


@pytest.fixture
def tree(tmp_path, monkeypatch):
    fw = tmp_path / "firmware" / "tinyg"
    (fw / "build").mkdir(parents=True)
    calls, state = [], {"tool": ("host:a", None), "fail": False}
    def run(cmd, **kw):
        calls.append(cmd)
        if state["fail"]:
            raise subprocess.CalledProcessError(2, cmd)
    monkeypatch.setattr(flash.subprocess, "run", run)
    monkeypatch.setattr(flash, "toolchain_id", lambda: state["tool"])
    return tmp_path, fw / "build" / ".toolchain", calls, state


def test_same_toolchain_incremental_other_toolchain_full(tree):
    root, stamp, calls, state = tree
    flash.build(root)
    assert "-B" in calls[-1] and stamp.read_text().strip() == "host:a"      # no stamp yet
    flash.build(root)
    assert "-B" not in calls[-1]                                             # same toolchain
    state["tool"] = ("image:sha256:1", "sha256:1")
    flash.build(root)
    assert calls[-1][0] == "docker" and "-B" in calls[-1] and "sha256:1" in calls[-1]
    state["tool"] = ("image:sha256:2", "sha256:2")                           # same tag, rebuilt image
    flash.build(root)
    assert "-B" in calls[-1] and stamp.read_text().strip() == "image:sha256:2"


def test_failed_build_invalidates_the_stamp(tree):
    root, stamp, calls, state = tree
    flash.build(root)                                                        # host build, stamp host:a
    state["tool"], state["fail"] = ("image:sha256:1", "sha256:1"), True
    with pytest.raises(subprocess.CalledProcessError):
        flash.build(root)                                                    # container build fails midway
    assert not stamp.exists()
    state["tool"], state["fail"] = ("host:a", None), False
    flash.build(root)                                                        # back on host: must rebuild fully
    assert "-B" in calls[-1] and stamp.read_text().strip() == "host:a"
