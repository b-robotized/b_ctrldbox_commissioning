"""Compare a device with the b»controlled box reference setup (docs/REFERENCE_SETUP.md).

All checks only read from the device.
"""

from __future__ import annotations

import ipaddress
import re
from dataclasses import dataclass
from pathlib import Path

from .apps import STANDARD_APPS, plan_updates
from .client import CtrlxClient, CtrlxError

OK, WARN, FAIL, INFO = "ok", "warn", "FAIL", "info"

BCTRLDBOX_APPS = (
    "ros2-base-jazzy",
    "zenohd-router-jazzy-binary",
    "b-controlled-box-rt-jazzy",
    "b-controlled-box-ui-jazzy",
)
BCTRLDBOX_BUNDLE = "b»Controlled Box"
BCTRLDBOX_TASK = "bCtrldBox"
BCTRLDBOX_PRIORITY = 28  # lower number = higher priority
ECAT_TASK = "ECATExtIOs"
ECAT_PRIORITY = 23
NON_RT_TASKS = ("schedBackground",)
MIN_FREE_MB = 1000
ZENOH_ENV_FILE = "zenoh-router/zenoh_router.env"
ZENOH_LOG_LEVELS = ("error", "warn")  # info and lower write to the logbook for each connection or message


@dataclass
class Check:
    status: str
    item: str
    detail: str


def check_zenoh_log_level(client: CtrlxClient) -> Check:
    """Check RUST_LOG of the Zenoh router: debug and trace load the non-real-time cores with logging."""
    item = "Zenoh router log level"
    try:
        text = client.active_configuration_file(ZENOH_ENV_FILE)
    except CtrlxError as error:
        return Check(WARN, item, f"not checked: {error}")
    if text is None:
        return Check(INFO, item, f"{ZENOH_ENV_FILE} not found (router not installed or not started yet)")
    match = None
    for line in text.splitlines():
        found = re.match(r"\s*(export\s+)?RUST_LOG\s*=\s*['\"]?([^'\"#]*)", line)
        if found:
            match = found
    if not match:
        return Check(INFO, item, f"RUST_LOG not set in {ZENOH_ENV_FILE} (zenohd default: info)")
    value = match.group(2).strip()
    levels = {part.rpartition("=")[2].strip().lower() for part in value.split(",") if part.strip()}
    fix = f"set export RUST_LOG='zenoh=warn,zenoh_transport=warn' in {ZENOH_ENV_FILE} and restart the router"
    if levels & {"trace", "debug"}:
        return Check(FAIL, item, f"{value}: floods the logbook and loads the non-real-time cores; {fix}")
    if not match.group(1):
        return Check(WARN, item, f"{value} without 'export': zenohd does not get it and uses info; {fix}")
    if not levels <= set(ZENOH_LOG_LEVELS):
        return Check(WARN, item, f"{value}; {fix}")
    return Check(OK, item, value)


def check_device(client: CtrlxClient, apps_dir: Path | None = None) -> list[Check]:
    results: list[Check] = []
    add = lambda status, item, detail="": results.append(Check(status, item, detail))
    installed = {p["name"]: (p.get("release") or {}).get("version") for p in client.packages()}

    # Device and mode
    info = client.system_info()
    add(INFO, "device", f"{info.get('hostname')}, ctrlX OS {info.get('ctrlxOsVersion')}, {info.get('architecture')}")
    add(INFO, "operation mode", client.mode())

    # System apps
    if apps_dir:
        try:
            _, system, _ = plan_updates(apps_dir, installed, ())
            if system:
                add(WARN, "system apps", "outdated: " + ", ".join(f"{i.app.name} {i.installed} -> {i.app.version}" for i in system))
            else:
                add(OK, "system apps", "same as newest set in the apps folder")
        except (FileNotFoundError, ValueError) as error:
            add(WARN, "system apps", f"not checked: {error}")
    else:
        add(INFO, "system apps", "not checked (no apps folder given)")

    # Standard Bosch apps
    missing = [a for a in STANDARD_APPS if a not in installed]
    add(FAIL if missing else OK, "standard apps", f"missing: {', '.join(missing)}" if missing else ", ".join(f"{a} {installed[a]}" for a in STANDARD_APPS))
    if "ctrlx-ixon-ixagent" in installed and "core22" not in installed:
        add(FAIL, "core22", "IXON agent needs core22")

    # b»controlled box apps and bundle
    missing = [a for a in BCTRLDBOX_APPS if a not in installed]
    add(FAIL if missing else OK, "b»controlled box apps", f"missing: {', '.join(missing)}" if missing else ", ".join(f"{a} {installed[a]}" for a in BCTRLDBOX_APPS))
    try:
        state = client.datalayer_read(f"framework/bundles/{BCTRLDBOX_BUNDLE}/state")["value"]
        add(OK if state == "Active" else FAIL, "RT bundle", state)
    except CtrlxError:
        add(FAIL, "RT bundle", "not loaded")

    if "zenohd-router-jazzy-binary" in installed:
        results.append(check_zenoh_log_level(client))

    # App settings and App Storage
    add(OK if client.package_settings().get("allowUnknownApps") else FAIL, "unknown apps allowed", "")
    stored = {p["name"] for p in client.app_storage_packages()}
    if not client.app_storage_active():
        add(WARN, "App Storage", "not active")
    else:
        missing = [a for a in STANDARD_APPS if a not in stored]
        add(WARN if missing else OK, "App Storage", f"missing: {', '.join(missing)}" if missing else "standard apps stored")

    # Time
    time = client.time_settings()
    servers = time.get("ntpServers") or []
    if not time.get("timeSynchronization") or not servers:
        add(FAIL, "NTP", "time synchronization off or no NTP servers")
    else:
        in_sync = client.ntp_in_sync()
        add(OK if in_sync else WARN, "NTP", f"{'in sync' if in_sync else 'NOT in sync'}, servers: {', '.join(servers)}")
    timezone = str(time.get("timezone", ""))
    add(WARN if timezone.startswith("UTC") else OK, "time zone", timezone + (" (set the time zone of the site)" if timezone.startswith("UTC") else ""))
    if time.get("rtcOk") is False:
        add(WARN, "real-time clock", "RTC not OK (battery?)")

    # Disk
    disk = client.data_disk()
    if disk:
        free = (disk["size"] - disk["used"]) / 1e6
        status = OK if free >= MIN_FREE_MB else (WARN if free >= 300 else FAIL)
        add(status, "data disk", f"{free:.0f} MB free of {disk['size'] / 1e6:.0f} MB (need >= {MIN_FREE_MB} MB for backups and updates)")

    # Real-time cores and tasks
    cores = client.cpu_settings().get("cores", {})
    rt_cores = sorted(int(name.replace("core", "")) for name, c in cores.items() if c.get("realtime"))
    add(OK if len(rt_cores) >= 2 else WARN, "real-time cores", f"{rt_cores}" + ("" if len(rt_cores) >= 2 else " (use 2 real-time cores)"))
    tasks = {n: t for n, t in client.scheduler_tasks().items() if n not in NON_RT_TASKS}
    box = tasks.get(BCTRLDBOX_TASK)
    if not box:
        add(WARN, "RT task", f"task {BCTRLDBOX_TASK} not found")
    else:
        core = box.get("affinity")
        add(OK if core in rt_cores else FAIL, "RT task core", f"{BCTRLDBOX_TASK}: core {core}, {box.get('cycletime')} us, priority {box.get('priority')}")
        shared = [n for n, t in tasks.items() if n != BCTRLDBOX_TASK and t.get("affinity") == core]
        free_rt = [c for c in rt_cores if c not in {t.get("affinity") for t in tasks.values()}]
        if shared:
            hint = f"; move them to real-time core {free_rt[0]}" if free_rt else ""
            add(WARN, "RT task separation", f"{', '.join(shared)} on the same core as {BCTRLDBOX_TASK}{hint}")
        else:
            add(OK, "RT task separation", f"{BCTRLDBOX_TASK} has core {core} alone")
        priority = box.get("priority")
        add(
            OK if priority == BCTRLDBOX_PRIORITY else WARN,
            "RT task priority",
            f"{BCTRLDBOX_TASK}: priority {priority}" + ("" if priority == BCTRLDBOX_PRIORITY else f" (use {BCTRLDBOX_PRIORITY})"),
        )
        ecat = tasks.get(ECAT_TASK)
        if ecat:
            ecat_ok = ecat.get("affinity") in rt_cores and ecat.get("affinity") != core and ecat.get("priority") == ECAT_PRIORITY
            add(
                OK if ecat_ok else WARN,
                "EtherCAT task",
                f"{ECAT_TASK}: core {ecat.get('affinity')}, priority {ecat.get('priority')}"
                + ("" if ecat_ok else f" (use the other real-time core, priority {ECAT_PRIORITY})"),
            )

    # Network: two ports with addresses in the same subnet
    seen: dict[ipaddress.IPv4Network, str] = {}
    conflicts = []
    for port, addresses in client.network_addresses().items():
        for entry in addresses:
            try:
                iface = ipaddress.ip_interface(entry.get("address", ""))
            except ValueError:
                continue
            if iface.version != 4 or iface.is_link_local:
                continue
            other = seen.get(iface.network)
            if other and other != port:
                conflicts.append(f"{other} and {port} in {iface.network}")
            seen.setdefault(iface.network, port)
    add(WARN if conflicts else OK, "network subnets", "; ".join(conflicts) if conflicts else "each subnet on one port")
    return results
