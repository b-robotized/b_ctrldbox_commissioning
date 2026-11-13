"""Prepare a ctrlX backup for a restore on another device.

A ctrlX backup (`*_ctrlx-backup.zip`) has the main file `ctrlx-setup.json`.
Other files are referenced with `"$path"`; only referenced files are restored.
Some sections belong to one device only (network addresses, host name,
certificates, licenses, users). They are removed before the backup is
applied to another device in `merge` mode, so the target keeps its own values.
"""

from __future__ import annotations

import json
import zipfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

SETUP_FILE = "ctrlx-setup.json"

# Sections that belong to the source device. They stay unchanged on the target.
DEVICE_SECTIONS = (
    "connectivity",          # IP addresses of XF10, XF50, XF51, ...
    "identityManagement",    # users and password hashes
    "licenseManagement",     # licenses are bound to the device
    "certificateManagement", # keys are encrypted per device
    "systemInfo",
    "typePlate",
)


@dataclass
class TransferPlan:
    kept: list[str] = field(default_factory=list)
    removed: list[str] = field(default_factory=list)
    apps: dict[str, str] = field(default_factory=dict)  # name -> version on the source
    app_files_in_backup: list[str] = field(default_factory=list)


def read_setup(backup: Path) -> dict[str, Any]:
    with zipfile.ZipFile(backup) as archive:
        return json.loads(archive.read(SETUP_FILE))


def plan(setup: dict[str, Any], keep: tuple[str, ...] = ()) -> TransferPlan:
    result = TransferPlan()
    for key in setup:
        if key.startswith("$"):
            continue
        if key in DEVICE_SECTIONS and key not in keep:
            result.removed.append(key)
        else:
            result.kept.append(key)
    if "hostname" in setup.get("system", {}) and "hostname" not in keep:
        result.removed.append("system.hostname")
    installed = setup.get("packageManagement", {}).get("installedApps", {})
    result.apps = {name: app.get("version", "") for name, app in installed.items()}
    result.app_files_in_backup = sorted(
        name for name, app in installed.items() if isinstance(app, dict) and "$path" in app
    )
    if "packageManagement" in result.kept:
        result.removed.append("packageManagement.installedApps")
    return result


def sanitize(backup: Path, target: Path, keep: tuple[str, ...] = ()) -> TransferPlan:
    """Write a copy of `backup` without device sections and without app files.

    Apps are installed separately (see `ctrlx-remote transfer apps`).
    """
    setup = read_setup(backup)
    result = plan(setup, keep)
    clean = {k: v for k, v in setup.items() if k.startswith("$") or k in result.kept}
    clean["$forceReboot"] = False
    if "system.hostname" in result.removed:
        clean["system"] = {k: v for k, v in clean["system"].items() if k != "hostname"}
    if "packageManagement" in clean:
        clean["packageManagement"] = {
            k: v for k, v in clean["packageManagement"].items() if k != "installedApps"
        }

    referenced = _referenced_paths(clean)
    target.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(backup) as source, zipfile.ZipFile(
        target, "w", compression=zipfile.ZIP_DEFLATED
    ) as out:
        out.writestr(SETUP_FILE, json.dumps(clean, indent=2))
        for info in source.infolist():
            name = info.filename
            if name in ("setup.v1.schema.json", "readme.md") or any(
                name == ref or name.startswith(ref.rstrip("/") + "/") for ref in referenced
            ):
                out.writestr(info, source.read(name))
    return result


def _referenced_paths(node: Any) -> set[str]:
    paths: set[str] = set()
    if isinstance(node, dict):
        if isinstance(node.get("$path"), str):
            paths.add(node["$path"])
        for value in node.values():
            paths |= _referenced_paths(value)
    elif isinstance(node, list):
        for value in node:
            paths |= _referenced_paths(value)
    return paths
