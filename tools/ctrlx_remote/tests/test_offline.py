"""Offline tests: no device needed."""

import io
import json
import tarfile
import zipfile
from pathlib import Path

from ctrlx_remote.apps import is_newer, plan_updates, read_app_file, version_key
from ctrlx_remote.transfer import sanitize


def make_app(path: Path, snap: str, archs=("amd64", "arm64")) -> Path:
    with tarfile.open(path, "w") as archive:
        for arch in archs:
            for suffix in (".snap", ".assert"):
                info = tarfile.TarInfo(f"public/snaps/{arch}/release/{snap}{suffix}")
                info.size = 0
                archive.addfile(info, io.BytesIO())
    return path


def test_version_compare():
    assert is_newer("4.6.6", "4.6.5")
    assert is_newer("20260824", "20260410")
    assert is_newer("6.12.40-ctrlx-59.3", "6.12.40-ctrlx-59.2")
    assert not is_newer("4.6.4", "4.6.4")
    assert is_newer("1.0", None)
    assert version_key("4.6.10") > version_key("4.6.9")


def test_read_app_file(tmp_path):
    app = read_app_file(make_app(tmp_path / "Device Admin-4.6.6.app", "rexroth-deviceadmin-4.6.6"),
                        ["rexroth-deviceadmin"])
    assert (app.name, app.version, app.architectures) == ("rexroth-deviceadmin", "4.6.6", ("amd64", "arm64"))
    core = read_app_file(make_app(tmp_path / "core22-20260410.app", "core22_20260410", ("arm64",)))
    assert (core.name, core.version, core.architectures) == ("core22", "20260410", ("arm64",))


def test_plan_updates(tmp_path):
    system = tmp_path / "X3_System_Apps_4.6.6"
    system.mkdir()
    (tmp_path / "X3_System_Apps_4.6.3").mkdir()
    package = tmp_path / "DC_App_Paket_4.6.3"
    package.mkdir()
    make_app(system / "Device Admin-4.6.6.app", "rexroth-deviceadmin-4.6.6")
    make_app(system / "Setup-4.6.4.app", "rexroth-setup-4.6.4")
    make_app(package / "Firewall-4.6.4.app", "rexroth-firewall-4.6.4")
    installed = {"rexroth-arch01-hw": "4.6.4", "rexroth-deviceadmin": "4.6.5", "rexroth-setup": "4.6.4"}
    dtype, sys_items, std_items = plan_updates(tmp_path, installed, ["rexroth-firewall"])
    assert dtype == "X3"
    assert [(i.app.name, i.reason) for i in sys_items] == [("rexroth-deviceadmin", "update")]
    assert [(i.app.name, i.reason) for i in std_items] == [("rexroth-firewall", "install")]


def test_sanitize(tmp_path):
    setup = {
        "$version": "1",
        "connectivity": {"ethernet": {}},
        "identityManagement": {"users": {}},
        "licenseManagement": {},
        "certificateManagement": {"$path": "certificateManagement"},
        "systemInfo": {}, "typePlate": {},
        "system": {"hostname": "box-a", "cpu": {"cores": {}}},
        "packageManagement": {"allowUnknownApps": True, "installedApps": {"x": {"version": "1"}}},
        "configurations": {"active": {"$path": "configurations/active"}},
    }
    source = tmp_path / "a.zip"
    with zipfile.ZipFile(source, "w") as z:
        z.writestr("ctrlx-setup.json", json.dumps(setup))
        z.writestr("configurations/active/b-controlled-box/scenario_controllers.yaml", "a: 1")
        z.writestr("certificateManagement/key.pem.aes", "secret")
    target = tmp_path / "b.zip"
    plan = sanitize(source, target)
    assert plan.apps == {"x": "1"}
    with zipfile.ZipFile(target) as z:
        names = z.namelist()
        clean = json.loads(z.read("ctrlx-setup.json"))
    assert "configurations/active/b-controlled-box/scenario_controllers.yaml" in names
    assert not any(n.startswith("certificateManagement") for n in names)
    for key in ("connectivity", "identityManagement", "licenseManagement", "certificateManagement"):
        assert key not in clean
    assert "hostname" not in clean["system"]
    assert clean["packageManagement"] == {"allowUnknownApps": True}
