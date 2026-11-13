"""Find app files in a local folder and compare them with installed apps.

A ctrlX `.app` file is a tar archive with one snap per architecture:
`public/snaps/<arch>/release/<snap-name>-<version>.snap` (+ `.assert`).
A plain `.snap` file has the name `<snap-name>_<version>[_<arch>].snap`.
"""

from __future__ import annotations

import re
import shutil
import subprocess
import tarfile
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

# Bosch apps that every b»controlled box gets (beside the system apps).
STANDARD_APPS = ("rexroth-ethercatmaster", "rexroth-firewall", "ctrlx-ixon-ixagent")

# Folder names in the Nextcloud app folder, per device type.
SYSTEM_APPS_PREFIX = {"X3": "X3_System_Apps_", "X5_X7_IPC": "X5_X7_IPC_System_Apps_"}
APP_PACKAGE_PREFIX = "DC_App_Paket_"

_APP_MEMBER = re.compile(r"public/snaps/(?P<arch>[^/]+)/release/(?P<file>.+)\.snap$")
_SNAP_FILE = re.compile(r"^(?P<name>[a-z0-9-]+)_(?P<version>.+?)(?:_(?P<arch>amd64|arm64))?(?:\(\d+\))?$")


@dataclass(frozen=True)
class AppFile:
    path: Path
    name: str  # snap name, e.g. rexroth-deviceadmin
    version: str
    architectures: tuple[str, ...]

    @property
    def package_id(self) -> str:
        return f"snap_{self.name}"


def version_key(version: str) -> tuple:
    """Sort key for versions like 4.6.6, 20260824, 6.12.40-ctrlx-59.3, 1.3.0-14-ui."""
    return tuple(int(part) if part.isdigit() else part for part in re.split(r"[.\-+_]", version))


def is_newer(candidate: str, installed: str | None) -> bool:
    if installed is None:
        return True
    try:
        return version_key(candidate) > version_key(installed)
    except TypeError:
        # Mixed number/text parts at the same position: compare as text.
        return candidate != installed and candidate > installed


def read_app_file(path: Path, known_names: Iterable[str] = ()) -> AppFile | None:
    """Read snap name, version, and architectures from a .app or .snap file."""
    if path.suffix == ".app":
        with tarfile.open(path) as archive:
            members = [m for m in (_APP_MEMBER.match(n) for n in archive.getnames()) if m]
        if not members:
            return None
        file_name = members[0]["file"]
        name, version = _split_name_version(file_name, known_names)
        return AppFile(path, name, version, tuple(sorted({m["arch"] for m in members})))
    if path.suffix == ".snap":
        meta = _snap_metadata(path)
        if meta:
            return AppFile(path, meta["name"], meta["version"], tuple(meta["architectures"]))
        # No unsquashfs: use the file name. Files without an arch in the name are not safe.
        match = _SNAP_FILE.match(path.stem)
        if not match:
            return None
        arch = match["arch"]
        return AppFile(path, match["name"], match["version"], (arch,) if arch else ())
    return None


def _snap_metadata(path: Path) -> dict | None:
    """Read name, version, and architectures from meta/snap.yaml (needs unsquashfs)."""
    if not shutil.which("unsquashfs"):
        return None
    result = subprocess.run(
        ["unsquashfs", "-cat", str(path), "meta/snap.yaml"],
        capture_output=True, text=True, check=False,
    )
    if result.returncode != 0:
        return None
    meta: dict = {"architectures": []}
    in_arch = False
    for line in result.stdout.splitlines():
        if in_arch and line.startswith("- "):
            meta["architectures"].append(line[2:].strip())
            continue
        in_arch = False
        if line.startswith(("name:", "version:")):
            key, value = line.split(":", 1)
            meta[key] = value.strip().strip("'\"")
        elif line.startswith("architectures:"):
            in_arch = True
    return meta if "name" in meta and "version" in meta else None


def _split_name_version(file_name: str, known_names: Iterable[str]) -> tuple[str, str]:
    """Split '<name>-<version>' or '<name>_<version>'.

    Snap names contain '-', so known names are checked first.
    """
    if "_" in file_name:
        name, version = file_name.split("_", 1)
        return name, version
    for name in sorted(known_names, key=len, reverse=True):
        if file_name.startswith(f"{name}-"):
            return name, file_name[len(name) + 1 :]
    match = re.match(r"^(?P<name>.+?)-(?P<version>\d.*)$", file_name)
    if not match:
        raise ValueError(f"Cannot read name and version from {file_name}")
    return match["name"], match["version"]


def scan(folder: Path, known_names: Iterable[str] = ()) -> list[AppFile]:
    known = list(known_names)
    apps = []
    for path in sorted(folder.glob("*")):
        if path.is_file():
            app = read_app_file(path, known)
            if app:
                apps.append(app)
    return apps


def newest_folder(root: Path, prefix: str) -> Path:
    candidates = [p for p in root.glob(f"{prefix}*") if p.is_dir()]
    if not candidates:
        raise FileNotFoundError(f"No folder {prefix}<version> in {root}")
    return max(candidates, key=lambda p: version_key(p.name[len(prefix) :]))


def device_type(installed_names: Iterable[str]) -> str:
    """X3 uses rexroth-arch01-*; X5/X7/IPC use rexroth-arch02-*."""
    names = set(installed_names)
    if any(n.startswith("rexroth-arch01") for n in names):
        return "X3"
    if any(n.startswith("rexroth-arch02") for n in names):
        return "X5_X7_IPC"
    raise ValueError("Unknown device type: no rexroth-arch01/arch02 hardware app installed")


@dataclass(frozen=True)
class UpdateItem:
    app: AppFile
    installed: str | None
    reason: str  # "update", "install"


def plan_updates(
    apps_root: Path,
    installed: dict[str, str],
    standard_apps: Iterable[str] = STANDARD_APPS,
) -> tuple[str, list[UpdateItem], list[UpdateItem]]:
    """Return (device type, system app updates, standard app installs/updates).

    `installed` maps snap name -> installed version.
    System apps: only the ones newer than installed.
    Standard apps: missing or newer than installed.
    """
    dtype = device_type(installed)
    known = set(installed) | set(standard_apps)
    system_dir = newest_folder(apps_root, SYSTEM_APPS_PREFIX[dtype])
    package_dir = newest_folder(apps_root, APP_PACKAGE_PREFIX)

    system = []
    for app in scan(system_dir, known):
        current = installed.get(app.name)
        if current is not None and is_newer(app.version, current):
            system.append(UpdateItem(app, current, "update"))
        elif current is None and app.name not in ("bare",):
            system.append(UpdateItem(app, None, "install"))

    candidates = scan(package_dir, known) + scan(apps_root, known)
    standard = []
    for name in standard_apps:
        matches = [a for a in candidates if a.name == name]
        if not matches:
            raise FileNotFoundError(f"No app file for {name} in {package_dir} or {apps_root}")
        app = max(matches, key=lambda a: version_key(a.version))
        current = installed.get(name)
        if is_newer(app.version, current):
            standard.append(UpdateItem(app, current, "install" if current is None else "update"))
    return dtype, system, standard
