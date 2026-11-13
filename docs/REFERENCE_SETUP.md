# Reference setup of a b»controlled box

This page defines a good setup of a ctrlX CORE with **b»controlled box**.
Every commissioning ends with a device that meets these points.
Check a device with:

```bash
ctrlx-remote --env-file <box>.env check [<apps-folder>]
```

The check only reads from the device.
`FAIL` means: fix before operation. `warn` means: fix when possible, or document why not.

## 1. Software

| # | Point | Check | How to fix |
|---|---|---|---|
| 1.1 | System apps on the tested ctrlX OS version (see [REQUIRED_SNAPS.md](REQUIRED_SNAPS.md)) | `system apps` | `ctrlx-remote system-apps update <apps-folder>`, before you install b»controlled box |
| 1.2 | Standard Bosch apps installed: EtherCAT Master, Firewall, IXON | `standard apps` | `system-apps update` installs them |
| 1.3 | `core22` installed when IXON is installed (newest: 20260824) | `core22` | `system-apps update --extra core22-<version>.app` |
| 1.4 | b»controlled box apps installed: `ros2-base-jazzy`, `zenohd-router-jazzy-binary`, `b-controlled-box-rt-jazzy`, `b-controlled-box-ui-jazzy` (newest) | `b»controlled box apps` | `apps install`, in this order |
| 1.5 | RT bundle `b»Controlled Box` is `Active` | `RT bundle` | Check the logbook; install the RT app again |
| 1.6 | "Allow installation from unknown sources" is on | `unknown apps allowed` | `Settings > Apps > Settings` |
| 1.7 | App Storage active and contains the standard apps and the current system apps | `App Storage` | `system-apps stage <apps-folder>` |

## 2. Time

| # | Point | Check | How to fix |
|---|---|---|---|
| 2.1 | Time synchronization on, with NTP servers in the local network, and the device in sync | `NTP` | `ctrlx-remote time set --servers <servers> --timezone <zone>` |
| 2.2 | Time zone of the site, not UTC | `time zone` | same command |

NTP servers: the ROS 2 PC and the commissioning container on the `XF10` network (default `192.168.28.201` and `192.168.28.202`, see [SETUP_NTP_SERVER.md](SETUP_NTP_SERVER.md)).
Add a public server (for example `ntp.ubuntu.com`) as fallback if the device has internet access.
ROS 2 time stamps, logs, and the logbook of the device can only be compared when all devices use the same time source.

## 3. Real-time cores and tasks

| # | Point | Check | How to fix |
|---|---|---|---|
| 3.1 | Two CPU cores are real-time cores (on a 4-core device: cores 2 and 3) | `real-time cores` | `Settings > System > CPU` (needs a restart) |
| 3.2 | The b»controlled box task (`bCtrldBox`) runs on a real-time core | `RT task core` | `Settings > Scheduler > Configuration` |
| 3.3 | **The b»controlled box task has its real-time core alone.** Other real-time tasks (for example `ECATExtIOs` for EtherCAT I/O, PLC tasks) run on the other real-time core. | `RT task separation` | `Settings > Scheduler > Configuration`: set the core (affinity) of the other tasks to the free real-time core; the change becomes active at the next change from SERVICE to SETUP |

If two cyclic tasks share one core, the task with the lower priority must wait for the other.
This adds jitter to the control loop of the robot.

## 4. Disk and network

| # | Point | Check | How to fix |
|---|---|---|---|
| 4.1 | At least 1 GB free on the data disk | `data disk` | Remove old apps from App Storage (`storage-remove`); on an X3 (2.9 GB) keep only the needed apps |
| 4.2 | Each IP subnet is on one port only | `network subnets` | Remove the second address or DHCP on one port |
| 4.3 | Commissioning port `XF10`: fixed address in `192.168.28.0/24` (default `192.168.28.28`; devices from b»robotized: the address in the delivery documents, see [SETUP_CTRLX.md](SETUP_CTRLX.md)); robot port (for example `XF12`): fixed address in the robot network | — | `Settings > Network Interfaces` |

## 5. Backup

| # | Point |
|---|---|
| 5.1 | A full backup after commissioning, stored outside the device, in a folder that the customer controls ([BACKUP_AND_TRANSFER.md](BACKUP_AND_TRANSFER.md)) |
| 5.2 | The app files of the delivery (`.snap`, `.app`) stored with the backup |

## Open points

- The priority of the `bCtrldBox` task is not the same on all devices (15 on a ctrlX CORE X7, 29 on X3 devices). A common value is not defined yet.
