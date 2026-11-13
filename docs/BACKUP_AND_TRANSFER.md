# Backup, restore, and transfer of a ctrlX CORE setup

This page explains the ctrlX backup format and how to back up, restore, and transfer a **b»controlled box** setup.
For the step-by-step move of an application to a new device, see [MOVE_TO_ANOTHER_BOX.md](MOVE_TO_ANOTHER_BOX.md).

## Make backups

**Make a full backup after commissioning and after every change.**
**Keep the backups in a folder that you control (not only on the device).**
**If you need support from b»robotized, give us this folder.**

With the web UI: `Settings > Backup & Restore > Create backup`.

With the tool:

```bash
ctrlx-remote --env-file <box>.env backup <folder>                 # full backup
ctrlx-remote --env-file <box>.env backup <folder> --without-apps  # without app files
```

The device first writes the backup to its own data disk, then the tool downloads it and deletes it on the device.
A full backup contains all installed apps and needs up to 1 GB free space on the device.
The ctrlX CORE X3 has a data disk of only 2.9 GB.
If there is not enough space, the backup fails with "no space left on device", and other apps can also fail to save their settings.
Then use `--without-apps`: this backup contains all settings and configurations and the list of installed apps, but no app files.
Keep the app files (`.snap`, `.app`) of each delivery in the same folder as the backup.

REST:

| Action | Call |
|---|---|
| Create | `POST /setup/api/v1/create` with `{}` (full) or `{"packageManagement": {"$content": "items"}}` (without app files). The response has no body; the task ID is in the `Location` header (`/tasks/create_<id>`). |
| Task state | `GET /setup/api/v1/tasks/<id>`; when `state` is `done`, `fileName` is the backup file. |
| Download | `GET /setup/api/v1/setups/<fileName>` |
| Delete on device | `DELETE /setup/api/v1/setups/<fileName>` |
| Free space | `GET /storage/api/v1/media` (`ubuntu-data`: `size`, `used`) |

## Backup format

A backup is a zip file (`<date>_<time>_ctrlx-backup.zip`):

| Path | Content |
|---|---|
| `ctrlx-setup.json` | Main file. Describes the device: apps, network, users, licenses, time, SSH, CPU cores, and more. |
| `setup.v1.schema.json` | JSON schema of `ctrlx-setup.json`; editors such as VS Code use it for editing support. |
| `configurations/active/` | Active configuration of all apps (Solutions app). For b»controlled box: `b-controlled-box/` with `scenario_controllers.yaml`, Zenoh configuration, and robot-specific files. |
| `packageManagement/installedApps/` | App files (only in a full backup). |
| `certificateManagement/` | Device certificates and keys. |

`ctrlx-setup.json` references other files with `"$path"`.
Only referenced files are restored.
See the `readme.md` in each backup.

## Restore on the same device

With the web UI: `Settings > Backup & Restore > Restore`.

REST: `POST /setup/api/v1/apply` (multipart) with the field `setup` (zip file) and `mode`:

- `merge` (default): settings that are not in the file stay unchanged.
- `restore`: settings that are not in the file are reset to their default values.

## Transfer to another device

Some parts of a backup belong to one device only.
If you apply them to another device, it gets the IP addresses, host name, or users of the source device.
Before a transfer, remove these sections:

| Section | Why |
|---|---|
| `connectivity` | IP addresses of `XF10`, `XF50`, `XF51`; two devices with the same IP address conflict. |
| `system.hostname` | Each device has its own host name. |
| `identityManagement` | Users and password hashes; keep the users of the target device. |
| `licenseManagement` | Licenses are bound to one device. |
| `certificateManagement` | Keys are encrypted per device. |
| `systemInfo`, `typePlate` | Information about the hardware of the source device. |

These sections stay: active configuration, `allowUnknownApps`, CPU core settings (real-time cores), time and NTP, hosts, proxy, SSH settings, network shares.

The tool does this:

```bash
ctrlx-remote transfer prepare <backup.zip> <clean.zip>          # offline; shows what stays and what is removed
ctrlx-remote --env-file <target>.env transfer apps <backup.zip> --snaps-dir <folder>
ctrlx-remote --env-file <target>.env transfer apply <clean.zip>  # merge mode
```

- `prepare` writes a copy of the backup without the device sections and without the app list.
  Use `--keep identityManagement` to keep a section.
- `apps` installs the non-system apps of the source device on the target device.
  It searches `<folder>` (and its subfolders) for files with the same name, version, and architecture.
  It reads the architecture from the snap metadata, not from the file name.
  Update the system apps of the target device first (`ctrlx-remote system-apps update`).
- `apply` switches the target device to SERVICE and applies the cleaned backup in `merge` mode.

### Check after the transfer

Make a backup of the target device (`--without-apps` is sufficient) and compare `configurations/active/` with the backup of the source device.
The files in `b-controlled-box/` must be the same.
These files are different by design, because ctrlX OS writes device data into them:

- `configuration.json` (device name),
- `scheduler/autoconfig.json` (internal task IDs),
- `trace/enablingstates_*.json` (serial number).

### Known limits

- The source and the target device must have the same architecture (X3 = `arm64`, X5/X7/IPC = `amd64`) for the app files; the configuration itself does not depend on it.
- Use the same ctrlX OS version on both devices.
- EtherCAT configurations can contain device descriptions of the source setup; check the EtherCAT Master configuration after the transfer.
- Licenses are not transferred; install the license of the target device.

## Update of Automation Core when b»controlled box RT is installed

An update of the Automation Core app copies its data folder, which contains the unpacked b»controlled box RT bundle (about 1 GB on an X3).
If the free space is not sufficient, the update fails with "no space left on device"; ctrlX OS keeps the old version.
Then:

1. Make a backup (`--without-apps` is sufficient).
2. Uninstall `b-controlled-box-rt-jazzy`. Its configuration in the Solutions app stays.
3. Update Automation Core.
4. Install the same `b-controlled-box-rt-jazzy` file again.

To avoid this, update the system apps before you install b»controlled box (see [SETUP_CTRLX_REMOTE.md](SETUP_CTRLX_REMOTE.md#4-update-the-system-apps)).
