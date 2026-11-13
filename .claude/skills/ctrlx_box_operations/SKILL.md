---
name: ctrlx_box_operations
description: Check, update, back up, restore, and transfer b»controlled boxes (ctrlX CORE) over the ctrlX REST API with the ctrlx-remote tool of this repository — system app updates via App Storage, backups, moving a setup to another device, installing b»controlled box apps. Load when asked to update, back up, restore, transfer, clone, check, or commission a ctrlX CORE / b»controlled box.
---

# ctrlX box operations

Use the `ctrlx-remote` tool in `tools/ctrlx_remote/`.
Details are in the docs; read the one you need, do not guess:

| Task | Doc |
|---|---|
| Setup and update steps, REST calls | `docs/SETUP_CTRLX_REMOTE.md` |
| Apps and versions | `docs/REQUIRED_SNAPS.md` |
| Backup format, restore, transfer | `docs/BACKUP_AND_TRANSFER.md` |
| Move to another device | `docs/MOVE_TO_ANOTHER_BOX.md` |
| API facts and known issues | `docs/CTRLX_REST_API_NOTES.md` |
| Commands | `tools/ctrlx_remote/README.md` |

## Setup

```bash
pip install "./tools/ctrlx_remote[mcp]"
```

One env file per device (`CTRLX_IP`, `CTRLX_USER`, `CTRLX_PASSWORD`, optional `CTRLX_SOURCE_ADDRESS`), `chmod 600`, outside the repository.
Ask the user to create it. Never write, print, or commit passwords.
Below: `CR="ctrlx-remote --env-file <box>.env"`.

If the device does not answer but the web UI works in a browser: check `ip route get <ip>` for a VPN route, then set `CTRLX_SOURCE_ADDRESS`.

## Rules

- Start read-only: `$CR info`, `$CR apps list`, `$CR system-apps check <apps-folder>`. Report, then ask.
- Ask the user before every change: install, update, remove, mode change, apply a backup, remove from App Storage, restart. An approval covers the plan you showed, not extra steps.
- Never switch to OPERATING unless the user asks for that device. Leave devices in SERVICE and say so.
- Make a backup before a change. Never commit backups, app files, or licenses.
- Check free disk space first (`$CR info`); the X3 has 2.9 GB.
- Long tasks (updates take minutes, restarts 2–3 minutes): run them in the background and watch the log.

## Backup

```bash
$CR backup <dir>                 # full (up to 1 GB free needed on the device)
$CR backup <dir> --without-apps  # settings, configurations, app list
```

Check: `unzip -tq <file>`; `configurations/active/b-controlled-box/` must be in it.

## Update (part of every commissioning)

```bash
$CR system-apps check  <apps-folder>
$CR -y system-apps update <apps-folder> [--extra <core22.app>]   # after approval
```

- The tool uses the newest version folders, uploads to App Storage, installs in the release-note order, waits through restarts, and returns to SERVICE.
- It is safe to run again; it skips what is up to date.
- IXON needs `core22`; pass it with `--extra` if it is not installed.
- Automation Core update fails with "no space left" when b»controlled box RT is installed: backup → uninstall RT → update → install the same RT file again → compare `configurations/` of a new backup with the old one. Prefer: update system apps before installing RT.

## Install b»controlled box apps

```bash
$CR -y apps install <ros2-base.snap> <zenoh.snap> <rt.snap> <ui.snap>   # in this order, SERVICE mode
```

The tool checks the architecture from the snap metadata and updates apps that are already installed.

## Transfer device A → device B

```bash
$CR_A backup <dir>
ctrlx-remote transfer prepare <A.zip> <clean.zip>
$CR_B -y system-apps update <apps-folder>
$CR_B -y transfer apps <A.zip> --snaps-dir <folder-with-snaps>
$CR_B -y transfer apply <clean.zip>
```

Then check: bundle `framework/bundles/b»Controlled Box/state` is `Active`; `$CR_B info` shows B's own IP and host name; `configurations/active/b-controlled-box/` in a new backup of B equals A.
Expected differences: `configuration.json`, `scheduler/autoconfig.json`, `trace/enablingstates_*.json`.

## After the work

Tell the user the mode of each device, the versions, and where the backups are.
Recommend: full backup after commissioning and after every change, in a folder the customer controls.
