# ctrlX OS REST API: notes and known issues

This page collects facts about the ctrlX OS REST API that we found during commissioning of **b»controlled box** and that are not (or not correctly) in the public API description.
Tested with ctrlX OS 4.6 on ctrlX CORE X3, November 2026.
The `ctrlx-remote` tool ([tools/ctrlx_remote](../tools/ctrlx_remote/README.md)) implements all of them.

Public API description: <https://boschrexroth.github.io/rest-api-description/ctrlx-automation/ctrlx-core>.

## Authentication

- `POST /identity-manager/api/v2/auth/token` with `{"name": "<user>", "password": "<password>"}` returns `access_token`.
- Send it as `Authorization: Bearer <token>`.
- The token expires; on `401`, log in again.
- The device uses a self-signed certificate (`curl -k`, or give the device CA).

## Apps (Package Manager, `/package-manager/api/v1`)

| Action | Call | Note |
|---|---|---|
| Installed apps | `GET /packages` | Version is in `release.version`. |
| Settings | `GET /settings`, `PUT /settings` `{"allowUnknownApps": true}` | `PUT` returns an empty body. |
| Install a new app from a file | `POST /packages` (multipart `file`) | Fails with "App already installed" for an installed app. |
| Install or update from a file | 1. `POST /batch-upload` (multipart field **`files`**) 2. wait until `GET /batch-upload/<id>` has `state` not `pending`/`running` 3. `POST /tasks` `{"action": "update", "parameters": {"id": "snap_<name>", "version": "<v>", "location": "batch://<id>"}}` (`install` for a new app) | With the wrong field name (`file`) the batch is empty, and the task still reports `done`. |
| Install a whole batch | `POST /tasks` `{"action": "batch", "parameters": {"location": "batch://<id>"}}` | The public description shows `{"id": ...}`; the device uses `location`. |
| Uninstall | `POST /tasks` `{"action": "uninstall", "parameters": {"id": "snap_<name>"}}` | The configuration of the app in the Solutions app stays. |
| Task state | `GET /tasks/<id>` | `state`: `pending`, `running`, `done`, `failed`; error text in `result.detail`. |

Install and update tasks need operation mode **SERVICE**; otherwise the device answers `409 system must be in state 'SERVICE'`.

## App Storage

App Storage keeps app files on the device (`Settings > Apps > App Storage`, shown as "Available apps").
These calls are not in the public description (Apps Management API 1.15.0); the ctrlX OS 4.6 web UI uses them.

| Action | Call |
|---|---|
| Active? | `GET /package-manager/api/v1/repositories` contains `id: control_repository` (`null` if not active) |
| Activate | `POST /package-manager/api/v1/repositories` with `{"id": "control_repository", "name": "Local storage", "path": "file:///var/snap/rexroth-deviceadmin/common/repositories/control_repository", "type": "snap", "writeable": true}` |
| List | `GET /package-manager/api/v1/repositories/control_repository/packages` |
| Upload | `POST /package-manager/api/v1/repositories/control_repository/packages` (multipart field **`fileName`**) |
| Install / update from it | `POST /package-manager/api/v1/tasks` with `"location": "repository://control_repository"` |
| Remove | `DELETE /package-manager/api/v1/repositories/control_repository/packages/snap_<name>/releases/<version>` |

App Storage keeps only the part of a `.app` file for the device architecture.

## Operation mode

- Read: `GET /automation/api/v2/nodes/scheduler/admin/state` → `{"value": {"state": "SERVICE"}}`.
- Write: `PUT` on the same node with `{"type": "object", "value": {"state": "SETUP"}}`.
- Change one step at a time: OPERATING ↔ SETUP ↔ SERVICE (as the web UI does).
- After a restart, the device starts in its startup mode (usually OPERATING), also during an update.

## Time and NTP

- Read: `GET /system/api/v1/time/settings` (`timezone`, `timeSynchronization`, `ntpServers`, `rtcOk`, poll intervals).
- Write: `PUT` on the same path; response `204` with no body. Without `timestamp` the device does not change its clock by hand.
- Sync state: `GET /automation/api/v2/nodes/system/time/synchronization/status/in-sync` (`true` a few seconds after a valid server is set).
- A new device has time synchronization off, no NTP servers, and time zone UTC.

## Real-time cores and scheduler tasks

- Real-time cores: `GET /system/api/v1/settings/cpu` (`cores.coreN.realtime`).
- Tasks: browse `scheduler/tasks` in the Data Layer; `scheduler/tasks/<name>/properties` has `cycletime` (µs), `priority`, and `affinity` (core).
- The b»controlled box task is `bCtrldBox`; the EtherCAT I/O task is `ECATExtIOs`; `schedBackground` is not real-time.

## Backup and restore (Setup app, `/setup/api/v1`)

- `POST /create` returns `201` with **no body**; the task ID is in the `Location` header (`/tasks/create_<id>`).
- The device writes the backup to its own data disk first (`/var/snap/rexroth-setup/common/_setups/`), then you download it with `GET /setups/<fileName>`; delete it with `DELETE /setups/<fileName>`.
- Without app files: body `{"packageManagement": {"$content": "items"}}`.
- Restore: `POST /apply` (multipart `setup`, `mode` = `merge` or `restore`).

See [BACKUP_AND_TRANSFER.md](BACKUP_AND_TRANSFER.md) for the format.

## System app update

- Order (ctrlX OS 4.6.6 release notes): install all system apps together, or snapd → core24 → Device Admin → all other apps.
- An update of `core24` restarts the device (about 2–3 minutes). An update of Device Admin restarts the web server.
  During the restart the device does not answer, then answers `502 Waiting for app to start`. Poll the task until it answers again.
- After the restart the device is in its startup mode; switch to SERVICE before the next install.
- The IXON agent (`ctrlx-ixon-ixagent` 0.1.21) needs the base app `core22`, which is not in the ctrlX OS 4.6.6 system app set.

## Disk space (ctrlX CORE X3)

- The data disk (`ubuntu-data`) has only 2.9 GB: `GET /storage/api/v1/media`.
- A full backup needs up to 1 GB free on the device.
- An update of Automation Core copies its data folder to the new revision. With b»controlled box RT installed, this folder contains the unpacked RT bundle with all ROS 2 libraries (about 1 GB). With too little space the update fails with "no space left on device", and ctrlX OS keeps the old version.
  Update the system apps before you install b»controlled box RT, or uninstall RT, update, and install RT again.
- With a full disk, apps cannot save settings (for example `Scheduler settings: ... No space left on device`).
- snapd keeps the previous revision of an updated app until the next update.

## Snap files

- Read name, version, and architecture from the snap itself: `unsquashfs -cat <file>.snap meta/snap.yaml`.
  A file name without an architecture does not tell you which one it is; we found amd64 files named `ros2-base-jazzy_3.1.0.snap` and `zenohd-router-jazzy-binary_1.0.snap`.
- A `.app` file is a tar archive with `public/snaps/<arch>/release/<name>-<version>.snap` (or `<name>_<version>.snap`) for one or more architectures.

## Device API documentation

- The device serves Swagger UI at `https://<ip>/doc/api`; it loads a description with `?url=<path>`.
  Unknown paths return the web UI start page (HTML), not `404`.

## Client side

- `urllib3-future` (it can replace `urllib3` in Python) switches to HTTP/3 after the first response and fails on the device certificate (`ValueError ... fromhex()`). Switch HTTP/3 off or use the standard `urllib3`.
- A VPN route can cover the network of the device and send traffic away from the local port. Check with `ip route get <ip>`; send from the local IP address (`curl --interface`, `CTRLX_SOURCE_ADDRESS`).
