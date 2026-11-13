# Remote setup of the ctrlX CORE over the REST API

This guide describes how to commission a ctrlX CORE for **b»controlled box** from your PC, without the web UI.
It uses the `ctrlx-remote` tool from [`tools/ctrlx_remote`](../tools/ctrlx_remote/README.md).
The tool uses the ctrlX OS REST API, which is the same API the web UI uses.
Each step also shows the REST call, so you can use `curl` or your own scripts.

For the setup with the web UI, see [SETUP_CTRLX.md](SETUP_CTRLX.md).

## 1. Before you start

1. Connect your PC to the commissioning port (`XF10`) of the ctrlX CORE.
2. Make sure that you can open `https://<device-ip>` in a web browser.
3. Install the tool:

   ```bash
   pip install ./tools/ctrlx_remote          # CLI
   pip install "./tools/ctrlx_remote[mcp]"   # CLI and MCP server
   ```

4. Create one settings file per device, for example `~/.config/b-ctrldbox/box1.env`, and make it readable only for you (`chmod 600`):

   ```bash
   CTRLX_IP=192.168.1.1
   CTRLX_USER=boschrexroth
   CTRLX_PASSWORD=<password>
   # Optional: local IP to send from, if a VPN route hides the device network.
   # CTRLX_SOURCE_ADDRESS=192.168.1.10
   ```

   Do not put this file into a git repository.

All commands below use `ctrlx-remote --env-file ~/.config/b-ctrldbox/box1.env <command>`.
In the examples this is shortened to `ctrlx-remote <command>`.
Before a write action the tool asks for confirmation; `-y` skips the question.

## 2. Check the device

```bash
ctrlx-remote info
```

The output shows the ctrlX OS version, the architecture (`arm64` for X3, `amd64` for X5/X7/IPC), the operation mode, and the free space on the data disk.

**Check the free space.**
The X3 has a data disk of only 2.9 GB.
A full backup is first written on the device and needs up to 1 GB.
If the disk is full, ctrlX OS cannot save configuration changes (for example scheduler settings).

REST: `POST /identity-manager/api/v2/auth/token` with `{"name": "<user>", "password": "<password>"}` gives a bearer token.
Use it as `Authorization: Bearer <token>` in all other calls.
`GET /system/api/v1/info`, `GET /storage/api/v1/media`.

## 3. Make a backup

Make a backup before you change the device.

```bash
ctrlx-remote backup ./backups                 # full backup with all apps
ctrlx-remote backup ./backups --without-apps  # settings and configurations only
```

The tool downloads the backup and deletes it on the device.
Use `--without-apps` if the data disk has less than 1 GB free.
See [BACKUP_AND_TRANSFER.md](BACKUP_AND_TRANSFER.md) for details.

## 4. Update the system apps

Update the ctrlX OS system apps to the version that b»controlled box is tested with (see [REQUIRED_SNAPS.md](REQUIRED_SNAPS.md)).
The update is part of the commissioning.

You need a folder with the ctrlX app files, with this layout:

```text
<apps-folder>/
├── X3_System_Apps_<version>/          # system apps for ctrlX CORE X3
├── X5_X7_IPC_System_Apps_<version>/   # system apps for ctrlX CORE X5, X7, and IPC
├── DC_App_Paket_<version>/            # other ctrlX apps (EtherCAT Master, Firewall, ...)
└── ctrlx-ixon-ixagent_<version>.app   # IXON remote access
```

The tool selects the folder with the highest version automatically.

```bash
ctrlx-remote system-apps check  <apps-folder>   # show what is outdated or missing
ctrlx-remote system-apps stage  <apps-folder>   # upload to App Storage only
ctrlx-remote system-apps update <apps-folder>   # upload, switch to SERVICE, install
```

`update` does these steps:

1. It compares the installed versions with the newest app files for the device type.
2. It uploads the newer system apps and the standard apps (EtherCAT Master, Firewall, IXON) to the **App Storage** of the device.
   The apps then stay on the device and you can install them again without a PC.
3. It switches the device to **SERVICE** mode.
4. It installs the apps in the order from the ctrlX OS release notes: snapd, core24, Device Admin, then all other apps.
5. It shows the installed versions.

An update of `core24` restarts the device.
The tool waits until the device is available again.
After the restart the device is in its startup mode (usually OPERATING), and the tool switches it to SERVICE again.

The IXON agent needs the base app `core22`.
If `core22` is not installed, add it with `--extra <path>/core22-<version>.app`.

REST:

| Action | Call |
|---|---|
| Installed apps | `GET /package-manager/api/v1/packages` |
| Activate App Storage | `POST /package-manager/api/v1/repositories` with `{"id": "control_repository", "name": "Local storage", "path": "file:///var/snap/rexroth-deviceadmin/common/repositories/control_repository", "type": "snap", "writeable": true}` |
| Upload to App Storage | `POST /package-manager/api/v1/repositories/control_repository/packages` (multipart, field `fileName`) |
| Install from App Storage | `POST /package-manager/api/v1/tasks` with `{"action": "update", "parameters": {"id": "snap_<name>", "version": "<version>", "location": "repository://control_repository"}}` (`install` for a new app) |
| Task state | `GET /package-manager/api/v1/tasks/<id>` |

The App Storage calls are not in the public OpenAPI description (Apps Management API 1.15.0); they are used by the ctrlX OS 4.6 web UI.

## 5. Install the b»controlled box apps

Apps that are not signed by Bosch Rexroth need the setting "Allow installation from unknown sources".
Set it in `Settings > Apps > Settings`, or with `PUT /package-manager/api/v1/settings` and `{"allowUnknownApps": true}`.

Install the standard set (see [REQUIRED_SNAPS.md](REQUIRED_SNAPS.md)) in SERVICE mode:

```bash
ctrlx-remote mode set SERVICE
ctrlx-remote apps install ros2-base-jazzy_<version>_arm64.snap \
  zenohd-router-jazzy-binary_<version>_arm64.snap \
  b-controlled-box-rt-jazzy_<version>_arm64.snap \
  b-controlled-box-ui-jazzy_<version>_arm64.snap
```

The tool checks that each file matches the architecture of the device.

REST: `POST /package-manager/api/v1/packages` (multipart, field `file`) uploads and installs one app.

## 6. Operation mode

```bash
ctrlx-remote mode get
ctrlx-remote mode set SETUP
ctrlx-remote mode set OPERATING
```

The tool changes the mode one step at a time: SERVICE ↔ SETUP ↔ OPERATING.
Before OPERATING it always asks for confirmation, because then controllers can command the robot.

REST: `PUT /automation/api/v2/nodes/scheduler/admin/state` with `{"type": "object", "value": {"state": "SETUP"}}`.
Read the mode with `GET` on the same node.

## 7. Other REST calls

`ctrlx-remote raw <METHOD> <path> [--body '<json>']` sends one request with a valid token.
Use it for diagnosis, for example:

```bash
ctrlx-remote raw GET /automation/api/v2/nodes/scheduler/admin/state
```

The REST API descriptions are at <https://boschrexroth.github.io/rest-api-description/ctrlx-automation/ctrlx-core>.

## Troubleshooting

- **No connection, but the web UI works in the browser**: a VPN can route the device network away from the local port.
  Check with `ip route get <device-ip>`.
  Set `CTRLX_SOURCE_ADDRESS` to your local IP on that network.
- **`ValueError ... fromhex()` from `urllib3`**: the package `urllib3-future` switches to HTTP/3 and fails on the device certificate.
  The tool switches HTTP/3 off; for your own scripts, use the standard `urllib3`.
- **"no space left on device" in a backup or update task**: free space on the data disk first, for example remove old apps from App Storage with `ctrlx-remote storage-remove <name>`.
