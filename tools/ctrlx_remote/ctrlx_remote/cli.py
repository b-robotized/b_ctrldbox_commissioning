"""Command line interface: `ctrlx-remote <command>`.

Connection settings come from an env file (--env-file) or the environment:
CTRLX_IP, CTRLX_USER, CTRLX_PASSWORD. Optional: CTRLX_SOURCE_ADDRESS.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Any

from .apps import STANDARD_APPS, plan_updates, read_app_file
from .client import CtrlxClient, CtrlxError, Credentials, read_env_file
from .transfer import plan as transfer_plan
from .transfer import read_setup, sanitize
from .reference import FAIL, WARN, check_device

# Install order from the ctrlX OS 4.6.6 release notes; other apps follow.
UPDATE_ORDER = (
    "snapd",
    "core24",
    "rexroth-deviceadmin",
    "rexroth-automationcore",
    "rexroth-setup",
    "rexroth-solutions",
    "rexroth-arch01-hw",
    "rexroth-arch01-kernel",
    "rexroth-arch02-hw",
    "rexroth-arch02-kernel",
    "core22",
)


def _log(message: str) -> None:
    print(f"{time.strftime('%H:%M:%S')} {message}", flush=True)


def _client(args: argparse.Namespace) -> CtrlxClient:
    credentials = Credentials.from_env(args.env_file)
    source = args.source_address
    if source is None and args.env_file:
        source = read_env_file(args.env_file).get("CTRLX_SOURCE_ADDRESS")
    source = source or os.environ.get("CTRLX_SOURCE_ADDRESS")
    return CtrlxClient(credentials, verify=args.ca_cert or False, source_address=source)


def _installed(client: CtrlxClient) -> dict[str, str | None]:
    return {p["name"]: (p.get("release") or {}).get("version") for p in client.packages()}


def _progress(name: str):
    last: list[Any] = [None]

    def report(task: dict[str, Any]) -> None:
        key = (task.get("state"), task.get("progress"))
        if key != last[0]:
            _log(f"  {name}: {key[0]} {key[1] if key[1] is not None else ''}")
            last[0] = key

    return report


def _confirm(args: argparse.Namespace, question: str) -> None:
    if args.yes:
        return
    if input(f"{question} [y/N] ").strip().lower() != "y":
        raise SystemExit("Stopped by user.")


# -- commands ----------------------------------------------------------------


def cmd_info(args: argparse.Namespace) -> None:
    client = _client(args)
    info = client.system_info()
    media = client.get_json("/storage/api/v1/media")
    data = next((m for m in media if m.get("label") == "ubuntu-data"), None)
    print(f"host:          {client.credentials.host}")
    print(f"hostname:      {info.get('hostname')}")
    print(f"ctrlX OS:      {info.get('ctrlxOsVersion')} ({info.get('operatingSystem')})")
    print(f"architecture:  {info.get('architecture')}")
    print(f"mode:          {client.mode()}")
    if data:
        free = (data["size"] - data["used"]) / 1e6
        print(f"data disk:     {data['used'] / 1e6:.0f} / {data['size'] / 1e6:.0f} MB used, {free:.0f} MB free")
    print(f"unknown apps:  {client.package_settings().get('allowUnknownApps', False)}")
    print(f"App Storage:   {'active' if client.app_storage_active() else 'not active'}")


def cmd_apps_list(args: argparse.Namespace) -> None:
    client = _client(args)
    for name, version in sorted(_installed(client).items()):
        print(f"{name:40s} {version}")
    stored = client.app_storage_packages()
    if stored:
        print("\nApp Storage:")
        for package in stored:
            versions = ", ".join(r["version"] for r in package.get("releases", []))
            print(f"{package['name']:40s} {versions}")


def cmd_apps_install(args: argparse.Namespace) -> None:
    client = _client(args)
    files = [Path(f) for f in args.files]
    arch = client.system_info()["architecture"]
    for path in files:
        app = read_app_file(path)
        if app and app.architectures and arch not in app.architectures:
            raise CtrlxError(f"{path.name} is for {app.architectures}, device is {arch}")
    _confirm(args, f"Install {len(files)} file(s) on {client.credentials.host} (mode must be SERVICE)?")
    if client.mode() != "SERVICE":
        raise CtrlxError("Device is not in SERVICE mode. Run `ctrlx-remote mode set SERVICE` first.")
    _log(f"upload {', '.join(p.name for p in files)}")
    client.install_files(files, _installed(client), timeout=1800, poll=5,
                         on_progress=_progress("install"))
    after = _installed(client)
    for path in files:
        app = read_app_file(path)
        if app:
            print(f"  {'ok' if after.get(app.name) == app.version else 'NOT INSTALLED':14s} {app.name} {after.get(app.name)}")


def cmd_system_apps(args: argparse.Namespace) -> None:
    """check: show what is outdated. stage: upload to App Storage. update: also install."""
    client = _client(args)
    installed = _installed(client)
    extra = [Path(p) for p in args.extra or []]
    dtype, system, standard = plan_updates(Path(args.apps_dir), installed, args.standard or STANDARD_APPS)
    items = system + standard
    _log(f"device type {dtype}, {len(system)} system app update(s), {len(standard)} standard app(s)")
    for item in items:
        print(f"  {item.reason:8s} {item.app.name:30s} {item.installed!s:22s} -> {item.app.version}")
    if args.action == "check":
        return

    _confirm(args, f"Upload {len(items) + len(extra)} app file(s) to App Storage on {client.credentials.host}?")
    client.activate_app_storage()
    stored = {p["name"]: {r["version"] for r in p.get("releases", [])} for p in client.app_storage_packages()}
    for path in [item.app.path for item in items] + extra:
        app = read_app_file(path)
        if app and app.version in stored.get(app.name, set()):
            _log(f"in App Storage: {path.name}")
            continue
        _log(f"upload {path.name}")
        client.app_storage_upload(path)
    if args.action == "stage":
        return

    stored = {p["name"]: p["releases"][0]["version"] for p in client.app_storage_packages()}
    wanted = {item.app.name for item in items} | {
        a.name for a in (read_app_file(p) for p in extra) if a
    }
    order = [n for n in UPDATE_ORDER if n in wanted] + sorted(wanted - set(UPDATE_ORDER))
    _confirm(args, f"Switch {client.credentials.host} to SERVICE and install {', '.join(order)}?")
    mode_before = client.mode()
    _log(f"mode {mode_before} -> {client.set_mode('SERVICE')}")
    for name in order:
        if client.mode() != "SERVICE":
            # A restart (core24, kernel) puts the device back in its startup mode.
            _log(f"device is in {client.mode()} after restart -> {client.set_mode('SERVICE')}")
        _log(f"{'update' if name in installed else 'install'} {name} {stored[name]}")
        client.install_from_app_storage(
            f"snap_{name}", stored[name], update=name in installed,
            timeout=1800, poll=5, on_progress=_progress(name),
        )
    after = _installed(client)
    for name in order:
        status = "ok" if after.get(name) == stored[name] else "NOT UPDATED"
        print(f"  {status:12s} {name:30s} {after.get(name)}")


def cmd_storage_remove(args: argparse.Namespace) -> None:
    client = _client(args)
    stored = {p["name"]: [r["version"] for r in p.get("releases", [])] for p in client.app_storage_packages()}
    targets = [(n, v) for n in args.names for v in stored.get(n, [])]
    if not targets:
        raise CtrlxError(f"Not in App Storage: {', '.join(args.names)}")
    _confirm(args, f"Remove from App Storage on {client.credentials.host}: {targets}?")
    for name, version in targets:
        client.app_storage_delete(f"snap_{name}", version)
        _log(f"removed {name} {version} from App Storage")


def cmd_mode(args: argparse.Namespace) -> None:
    client = _client(args)
    if args.action == "get":
        print(client.mode())
        return
    target = args.target.upper()
    if target == "OPERATING":
        _confirm(args, f"Switch {client.credentials.host} to OPERATING? Controllers can command the robot.")
    print(client.set_mode(target))


def cmd_backup(args: argparse.Namespace) -> None:
    client = _client(args)
    out = Path(args.out_dir)
    _log(f"create backup on {client.credentials.host}")
    task = client.create_backup(password=args.password, include_apps=not args.without_apps,
                                timeout=3600, poll=10,
                                on_progress=_progress("backup"))
    name = task["fileName"]
    target = out / f"{client.hostname()}_{name}"
    client.download_setup(name, target)
    _log(f"downloaded {target} ({target.stat().st_size / 1e6:.0f} MB)")
    if not args.keep_on_device:
        client.request("DELETE", f"/setup/api/v1/setups/{name}")
        _log("deleted backup file on the device")


def _snap_files(folder: Path, arch: str) -> dict[tuple[str, str], Path]:
    """(name, version) -> file, for .snap/.app files in folder (recursive) that fit arch."""
    files = {}
    for path in sorted(folder.rglob("*")):
        if path.suffix not in (".snap", ".app") or not path.is_file():
            continue
        app = read_app_file(path)
        if app and (not app.architectures or arch in app.architectures):
            files.setdefault((app.name, app.version), path)
    return files


def cmd_transfer(args: argparse.Namespace) -> None:
    backup = Path(args.backup)
    if args.action == "prepare":
        out = Path(args.out)
        result = sanitize(backup, out, keep=tuple(args.keep or ()))
        print(f"written: {out}")
        print(f"kept:    {', '.join(result.kept)}")
        print(f"removed: {', '.join(result.removed)}")
        print("apps on the source device (install them separately with `transfer apps`):")
        for name, version in sorted(result.apps.items()):
            print(f"  {name:40s} {version}")
        return

    client = _client(args)
    if args.action == "apps":
        source_apps = transfer_plan(read_setup(backup)).apps
        installed = _installed(client)
        arch = client.system_info()["architecture"]
        files = _snap_files(Path(args.snaps_dir), arch)
        missing, todo = [], []
        for name, version in sorted(source_apps.items()):
            if installed.get(name) == version or name.startswith(("rexroth-", "core", "snapd", "bare")):
                continue  # same version, or a system app (update those with system-apps)
            path = files.get((name, version))
            (todo if path else missing).append((name, version, path))
        # Runtime and middleware first: other apps connect to their content slots.
        first = ("ros2-base-jazzy", "zenohd-router-jazzy-binary", "b-controlled-box-rt-jazzy")
        todo.sort(key=lambda item: first.index(item[0]) if item[0] in first else len(first))
        for name, version, path in todo:
            print(f"  install {name:36s} {version:30s} {path}")
        for name, version, _ in missing:
            print(f"  MISSING {name:36s} {version:30s} no file in {args.snaps_dir}")
        if missing and not args.ignore_missing:
            raise CtrlxError("App files are missing; add them or use --ignore-missing")
        if not todo:
            return
        _confirm(args, f"Switch {client.credentials.host} to SERVICE and install {len(todo)} app(s)?")
        if not client.package_settings().get("allowUnknownApps"):
            client.set_package_settings(allowUnknownApps=True)
            _log("allowed installation from unknown sources")
        _log(f"mode -> {client.set_mode('SERVICE')}")
        for name, version, path in todo:
            _log(f"install {path.name}")
            client.install_files([path], installed, timeout=1800, poll=5, on_progress=_progress(name))
        return

    if args.action == "apply":
        _confirm(args, f"Apply {backup.name} to {client.credentials.host} in mode {args.mode}?")
        _log(f"mode -> {client.set_mode('SERVICE')}")
        task = client.apply_setup(backup, mode=args.mode, timeout=3600, poll=5,
                                  on_progress=_progress("apply"))
        for entry in task.get("protocol", []):
            if entry.get("severity") in ("WARNING", "ERROR"):
                print(f"  {entry['severity']}: {entry['message']}")
        _log("applied")


def cmd_check(args: argparse.Namespace) -> None:
    client = _client(args)
    results = check_device(client, Path(args.apps_dir) if args.apps_dir else None)
    for result in results:
        print(f"  {result.status:5s} {result.item:24s} {result.detail}")
    fails = sum(r.status == FAIL for r in results)
    warns = sum(r.status == WARN for r in results)
    print(f"{fails} failed, {warns} warning(s). Reference: docs/REFERENCE_SETUP.md")


def cmd_time(args: argparse.Namespace) -> None:
    client = _client(args)
    if args.action == "set":
        _confirm(args, f"Set NTP servers {args.servers} and time zone {args.timezone} on {client.credentials.host}?")
        client.set_time_settings(args.servers, args.timezone)
        for _ in range(30):  # first sync takes a few seconds
            if client.ntp_in_sync():
                break
            time.sleep(2)
    settings = client.time_settings()
    print(f"time:         {settings.get('timestamp')} {settings.get('timezone')}")
    print(f"NTP:          {'on' if settings.get('timeSynchronization') else 'off'}, servers: {', '.join(settings.get('ntpServers') or []) or '-'}")
    print(f"in sync:      {client.ntp_in_sync()}")
    print(f"RTC OK:       {settings.get('rtcOk')}")


def cmd_raw(args: argparse.Namespace) -> None:
    client = _client(args)
    body = json.loads(args.body) if args.body else None
    response = client.request(args.method.upper(), args.path, json=body)
    print(json.dumps(response.json(), indent=2) if response.content else response.status_code)


# -- parser ------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="ctrlx-remote", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--env-file", help="file with CTRLX_IP, CTRLX_USER, CTRLX_PASSWORD")
    parser.add_argument("--source-address", help="local IP to send from (VPN route workaround)")
    parser.add_argument("--ca-cert", help="CA file to verify the device certificate (default: no check)")
    parser.add_argument("-y", "--yes", action="store_true", help="do not ask before write actions")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("info", help="device, mode, disk, settings").set_defaults(func=cmd_info)

    apps = sub.add_parser("apps", help="installed apps and App Storage")
    apps_sub = apps.add_subparsers(dest="action", required=True)
    apps_sub.add_parser("list").set_defaults(func=cmd_apps_list)
    install = apps_sub.add_parser("install", help="upload and install .snap/.app files")
    install.add_argument("files", nargs="+")
    install.set_defaults(func=cmd_apps_install)

    storage = sub.add_parser("storage-remove", help="remove apps from App Storage (installed apps stay)")
    storage.add_argument("names", nargs="+", help="snap names, e.g. ros2-base-jazzy")
    storage.set_defaults(func=cmd_storage_remove)

    system = sub.add_parser("system-apps", help="check, stage, or update system and standard apps")
    system.add_argument("action", choices=["check", "stage", "update"])
    system.add_argument("apps_dir", help="folder with <type>_System_Apps_<v>/ and DC_App_Paket_<v>/")
    system.add_argument("--standard", nargs="*", help=f"standard apps (default: {' '.join(STANDARD_APPS)})")
    system.add_argument("--extra", nargs="*", help="more .app files, e.g. a base snap a standard app needs")
    system.set_defaults(func=cmd_system_apps)

    mode = sub.add_parser("mode", help="operation mode")
    mode_sub = mode.add_subparsers(dest="action", required=True)
    mode_sub.add_parser("get").set_defaults(func=cmd_mode)
    mode_set = mode_sub.add_parser("set")
    mode_set.add_argument("target", choices=["SERVICE", "SETUP", "OPERATING", "service", "setup", "operating"])
    mode_set.set_defaults(func=cmd_mode)

    backup = sub.add_parser("backup", help="create and download a full backup")
    backup.add_argument("out_dir")
    backup.add_argument("--password", help="encrypt secret parts (keys, passwords)")
    backup.add_argument("--keep-on-device", action="store_true")
    backup.add_argument("--without-apps", action="store_true",
                        help="leave out app files (small backup; for a nearly full disk)")
    backup.set_defaults(func=cmd_backup)

    transfer = sub.add_parser("transfer", help="move a setup from one device to another")
    transfer_sub = transfer.add_subparsers(dest="action", required=True)
    prepare = transfer_sub.add_parser("prepare", help="remove device sections from a backup (offline)")
    prepare.add_argument("backup")
    prepare.add_argument("out")
    prepare.add_argument("--keep", nargs="*", help="device sections to keep, e.g. identityManagement")
    prepare.set_defaults(func=cmd_transfer)
    tapps = transfer_sub.add_parser("apps", help="install the non-system apps of the backup")
    tapps.add_argument("backup")
    tapps.add_argument("--snaps-dir", required=True, help="folder with .snap/.app files (searched recursively)")
    tapps.add_argument("--ignore-missing", action="store_true")
    tapps.set_defaults(func=cmd_transfer)
    apply = transfer_sub.add_parser("apply", help="apply a prepared backup to the device")
    apply.add_argument("backup")
    apply.add_argument("--mode", choices=["merge", "restore"], default="merge")
    apply.set_defaults(func=cmd_transfer)

    check = sub.add_parser("check", help="compare the device with the reference setup (read-only)")
    check.add_argument("apps_dir", nargs="?", help="apps folder, to check the system app versions")
    check.set_defaults(func=cmd_check)

    time_cmd = sub.add_parser("time", help="time zone and NTP")
    time_sub = time_cmd.add_subparsers(dest="action", required=True)
    time_sub.add_parser("get").set_defaults(func=cmd_time)
    time_set = time_sub.add_parser("set")
    time_set.add_argument("--servers", nargs="+", required=True,
                          help="NTP servers, e.g. 192.168.28.201 192.168.28.202")
    time_set.add_argument("--timezone", required=True, help="IANA name, e.g. Europe/Berlin")
    time_set.set_defaults(func=cmd_time)

    raw = sub.add_parser("raw", help="send one REST request (for diagnosis)")
    raw.add_argument("method")
    raw.add_argument("path")
    raw.add_argument("--body")
    raw.set_defaults(func=cmd_raw)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        args.func(args)
    except CtrlxError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
