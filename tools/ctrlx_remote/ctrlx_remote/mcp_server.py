"""MCP server for ctrlX OS devices (stdio).

Devices are env files in a folder (default ~/.config/b-ctrldbox/), one per box:
`<name>.env` with CTRLX_IP, CTRLX_USER, CTRLX_PASSWORD, optional CTRLX_SOURCE_ADDRESS.
Tools take the device name (file name without .env).

Read-only tools are always available. Write tools (mode, install, update,
backup files on the device) need the --allow-write flag.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
from typing import Any

from mcp.server.fastmcp import FastMCP

from .apps import STANDARD_APPS, plan_updates, read_app_file
from .cli import UPDATE_ORDER
from .client import CtrlxClient, CtrlxError, Credentials, read_env_file
from .reference import check_device

server = FastMCP("ctrlx")
_config: dict[str, Any] = {"devices_dir": Path("~/.config/b-ctrldbox").expanduser(), "allow_write": False}


def _client(device: str) -> CtrlxClient:
    env_file = _config["devices_dir"] / f"{device}.env"
    if not env_file.is_file():
        raise CtrlxError(f"Unknown device {device}. Known: {', '.join(list_devices())}")
    values = read_env_file(str(env_file))
    return CtrlxClient(
        Credentials.from_env(str(env_file)),
        source_address=values.get("CTRLX_SOURCE_ADDRESS") or os.environ.get("CTRLX_SOURCE_ADDRESS"),
    )


def _require_write() -> None:
    if not _config["allow_write"]:
        raise CtrlxError("Write tools are disabled. Start the server with --allow-write.")


def _installed(client: CtrlxClient) -> dict[str, str | None]:
    return {p["name"]: (p.get("release") or {}).get("version") for p in client.packages()}


@server.tool()
def list_devices() -> list[str]:
    """Names of the configured devices."""
    return sorted(p.stem for p in _config["devices_dir"].glob("*.env"))


@server.tool()
def device_info(device: str) -> dict[str, Any]:
    """System info, operation mode, data disk usage, and App Storage state."""
    client = _client(device)
    info = client.system_info()
    media = client.get_json("/storage/api/v1/media")
    data = next((m for m in media if m.get("label") == "ubuntu-data"), {})
    return {
        "hostname": info.get("hostname"),
        "ctrlxOsVersion": info.get("ctrlxOsVersion"),
        "architecture": info.get("architecture"),
        "mode": client.mode(),
        "dataDiskUsedMB": round(data.get("used", 0) / 1e6),
        "dataDiskSizeMB": round(data.get("size", 0) / 1e6),
        "allowUnknownApps": client.package_settings().get("allowUnknownApps", False),
        "appStorageActive": client.app_storage_active(),
    }


@server.tool()
def list_apps(device: str) -> dict[str, Any]:
    """Installed apps and apps in App Storage, with versions."""
    client = _client(device)
    return {
        "installed": _installed(client),
        "appStorage": {
            p["name"]: [r["version"] for r in p.get("releases", [])] for p in client.app_storage_packages()
        },
    }


@server.tool()
def check_updates(device: str, apps_dir: str) -> dict[str, Any]:
    """Compare installed apps with the newest system apps and standard apps in apps_dir."""
    client = _client(device)
    dtype, system, standard = plan_updates(Path(apps_dir), _installed(client))
    row = lambda i: {"app": i.app.name, "installed": i.installed, "available": i.app.version,
                     "file": str(i.app.path)}
    return {"deviceType": dtype, "systemApps": [row(i) for i in system],
            "standardApps": [row(i) for i in standard], "standardAppNames": list(STANDARD_APPS)}


@server.tool()
def check_setup(device: str, apps_dir: str | None = None) -> list[dict[str, str]]:
    """Compare the device with the reference setup (docs/REFERENCE_SETUP.md). Read-only.
    status: ok, warn, FAIL, info."""
    results = check_device(_client(device), Path(apps_dir) if apps_dir else None)
    return [{"status": r.status, "item": r.item, "detail": r.detail} for r in results]


@server.tool()
def set_time(device: str, ntp_servers: list[str], timezone: str) -> dict[str, Any]:
    """Set NTP servers and time zone, switch on time synchronization. Needs --allow-write."""
    _require_write()
    client = _client(device)
    settings = client.set_time_settings(ntp_servers, timezone)
    return {**settings, "inSync": client.ntp_in_sync()}


@server.tool()
def read_datalayer(device: str, node: str) -> Any:
    """Read one Data Layer node, e.g. scheduler/admin/state."""
    return _client(device).datalayer_read(node)


@server.tool()
def set_mode(device: str, mode: str) -> str:
    """Switch the operation mode (SERVICE, SETUP, OPERATING). Needs --allow-write."""
    _require_write()
    return _client(device).set_mode(mode)


@server.tool()
def update_apps(device: str, apps_dir: str, extra_files: list[str] | None = None) -> dict[str, Any]:
    """Upload outdated system apps and missing standard apps to App Storage, switch to
    SERVICE, and install them in the order of the release notes. Needs --allow-write."""
    _require_write()
    client = _client(device)
    installed = _installed(client)
    _, system, standard = plan_updates(Path(apps_dir), installed)
    files = [i.app.path for i in system + standard] + [Path(f) for f in extra_files or []]
    client.activate_app_storage()
    for path in files:
        client.app_storage_upload(path)
    stored = {p["name"]: p["releases"][0]["version"] for p in client.app_storage_packages()}
    wanted = {a.name for a in (read_app_file(p) for p in files) if a}
    order = [n for n in UPDATE_ORDER if n in wanted] + sorted(wanted - set(UPDATE_ORDER))
    for name in order:
        # A restart (core24, kernel) puts the device back in its startup mode.
        if client.mode() != "SERVICE":
            client.set_mode("SERVICE")
        client.install_from_app_storage(f"snap_{name}", stored[name], update=name in installed,
                                        timeout=1800, poll=5)
    after = _installed(client)
    return {name: {"before": installed.get(name), "after": after.get(name)} for name in order}


@server.tool()
def create_backup(device: str, out_dir: str) -> str:
    """Create a full backup, download it to out_dir, delete it on the device. Needs --allow-write."""
    _require_write()
    client = _client(device)
    task = client.create_backup(timeout=3600, poll=10)
    target = Path(out_dir) / f"{client.hostname()}_{task['fileName']}"
    client.download_setup(task["fileName"], target)
    client.request("DELETE", f"/setup/api/v1/setups/{task['fileName']}")
    return str(target)


def main() -> None:
    parser = argparse.ArgumentParser(prog="ctrlx-mcp")
    parser.add_argument("--devices-dir", default="~/.config/b-ctrldbox")
    parser.add_argument("--allow-write", action="store_true")
    args = parser.parse_args()
    _config["devices_dir"] = Path(args.devices_dir).expanduser()
    _config["allow_write"] = args.allow_write
    server.run()


if __name__ == "__main__":
    main()
