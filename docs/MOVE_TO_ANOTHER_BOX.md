# Move the b»controlled box application to another ctrlX CORE

This guide moves a working **b»controlled box** application from ctrlX CORE **A** (source) to ctrlX CORE **B** (target), for example to replace a device or to set up a second, identical cell.
Device B keeps its own IP addresses, host name, users, and license.

There are two ways: with the web UI (section 2) or with the `ctrlx-remote` tool (section 3).
Background on the backup format: [BACKUP_AND_TRANSFER.md](BACKUP_AND_TRANSFER.md).

## 1. Checklist before the move

- [ ] A and B have the same architecture: X3 = `arm64`, X5/X7/IPC = `amd64` (web UI: `Settings > System Information`; tool: `ctrlx-remote info`).
- [ ] B has the same ctrlX OS version as A, or a newer tested version (see [REQUIRED_SNAPS.md](REQUIRED_SNAPS.md)).
- [ ] B has more than 1 GB free on the data disk.
- [ ] You have the app files of A: `ros2-base-jazzy`, `zenohd-router-jazzy-binary`, `b-controlled-box-rt-jazzy`, `b-controlled-box-ui-jazzy`, and robot-specific apps, in the versions that run on A, for the architecture of B.
- [ ] You have the license file for B (if a license is used).
- [ ] The robot is stopped. A and B are not connected to the robot at the same time with the same IP address.

## 2. With the web UI

### On device A

1. Open `https://<ip-of-A>` and log in.
2. Note the installed apps and versions: `Settings > Apps`.
3. Make a full backup: `Settings > Backup & Restore > Create backup`, then download the file.
   If the backup fails with "no space left on device", select a backup without app files.
4. Note the network settings of the robot port (for example `XF12`): `Settings > Network Interfaces`.

### Prepare the backup for device B

The backup contains settings that belong to device A only.
Open the zip file and edit `ctrlx-setup.json` (a text editor is sufficient; VS Code uses `setup.v1.schema.json` for help):

1. Delete these top-level sections: `connectivity`, `identityManagement`, `licenseManagement`, `certificateManagement`, `systemInfo`, `typePlate`.
2. In `system`, delete `hostname`.
3. In `packageManagement`, delete `installedApps` (you install the apps in the next step).
4. Delete the folder `certificateManagement/` from the zip.
5. Save the zip. Keep the folder structure: `ctrlx-setup.json` must be in the root of the zip, not in a subfolder.

### On device B

1. Open `https://<ip-of-B>` and log in.
2. Update the system apps to the tested ctrlX OS version (see [SETUP_CTRLX_REMOTE.md](SETUP_CTRLX_REMOTE.md#4-update-the-system-apps) or `Settings > Apps > Install from file`, all system apps together).
3. Set the operation mode to **SERVICE** (header of the web UI).
4. Allow apps from unknown sources: `Settings > Apps > Settings > Allow installation from unknown sources`.
5. Install the apps of A in this order with `Settings > Apps > Install from file`:
   1. `ros2-base-jazzy`
   2. `zenohd-router-jazzy-binary`
   3. `b-controlled-box-rt-jazzy`
   4. `b-controlled-box-ui-jazzy`
   5. robot-specific apps
6. Restore the prepared backup: `Settings > Backup & Restore > Restore`, select the prepared zip file, mode **merge**.
7. Set the IP address of the robot port as on A (`Settings > Network Interfaces`), if B replaces A in the same cell.
8. Install the license of B: `Settings > Licenses`.
9. Check the scheduler cycle time: `Settings > Scheduler > Configuration`.
10. Switch to **SETUP** and check that the b»controlled box bundle is active and the controllers start (b»ui or `ros2 control list_controllers`).

## 3. With the tool

Create one env file per device (see [SETUP_CTRLX_REMOTE.md](SETUP_CTRLX_REMOTE.md#1-before-you-start)), for example `a.env` and `b.env`.

```bash
# 1. Device A: backup
ctrlx-remote --env-file a.env backup ./move            # or --without-apps if the disk is too small

# 2. Offline: remove the sections of device A
ctrlx-remote transfer prepare ./move/<A-backup>.zip ./move/for-B.zip

# 3. Device B: system apps on the tested version
ctrlx-remote --env-file b.env system-apps update <apps-folder>

# 4. Device B: apps of A (same versions, architecture of B)
ctrlx-remote --env-file b.env transfer apps ./move/<A-backup>.zip --snaps-dir <folder-with-snaps>

# 5. Device B: configuration of A (merge mode)
ctrlx-remote --env-file b.env transfer apply ./move/for-B.zip
```

Step 4 shows each file before it installs it, and it stops if a file is missing.
It reads the architecture from the snap file itself.

Then continue with steps 7 to 10 of section 2.

## 4. Checklist after the move

- [ ] B has its own IP address and host name (`ctrlx-remote info`).
- [ ] The b»controlled box bundle is active: `ctrlx-remote raw GET "/automation/api/v2/nodes/framework/bundles/b»Controlled Box/state"` shows `Active`.
- [ ] The configuration is the same as on A: `configurations/active/b-controlled-box/` in a new backup of B has the same files as the backup of A.
- [ ] The ROS 2 PC connects to the Zenoh router of B (see [SETUP_ZENOH.md](SETUP_ZENOH.md)).
- [ ] The robot is reachable from B (`Settings > Network Diagnostics`).
- [ ] NTP is on and in sync, time zone of the site (`ctrlx-remote time get`).
- [ ] `ctrlx-remote check` shows no `FAIL` ([REFERENCE_SETUP.md](REFERENCE_SETUP.md)).
- [ ] The controllers are active in SETUP; switch to OPERATING only when the cell is ready.
- [ ] Make a full backup of B and store it with the backup of A.

## 5. Device A after the move

- If A stays in use for another cell: change its robot port IP address or disconnect it, so that A and B do not conflict.
- If A is out of service: keep the backup of A. To reset A, use `Settings > Reset` (all settings and apps are deleted).
