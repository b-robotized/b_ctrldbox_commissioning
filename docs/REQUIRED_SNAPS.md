# Required apps (snaps) for b»controlled box

This page lists the apps that every ctrlX CORE with **b»controlled box** needs.
All apps exist for `arm64` (ctrlX CORE X3) and `amd64` (ctrlX CORE X5, X7, and IPC).
Use the file that matches the architecture of your device; `ctrlx-remote info` shows it.

## Tested versions

| Component | Version |
|---|---|
| ctrlX OS | 4.6.6 |
| b»controlled box RT | 1.7.x (Jazzy) |
| ROS 2 | Jazzy |

## ctrlX system apps

System apps come with ctrlX OS.
Update them to the versions of the tested ctrlX OS release before you install b»controlled box.
`ctrlx-remote system-apps update` does this, see [SETUP_CTRLX_REMOTE.md](SETUP_CTRLX_REMOTE.md#4-update-the-system-apps).

| App | Snap name | ctrlX OS 4.6.6 |
|---|---|---|
| snapd | `snapd` | 2.75.2 |
| Base | `core24` | 20260824 |
| Device Admin | `rexroth-deviceadmin` | 4.6.6 |
| Automation Core | `rexroth-automationcore` | 4.6.5 |
| Setup | `rexroth-setup` | 4.6.4 |
| Solutions | `rexroth-solutions` | 4.6.6 |
| Hardware Support X3 / X7 | `rexroth-arch01-hw` / `rexroth-arch02-hw` | 4.6.4 / 4.6.6 |
| Realtime Kernel X3 / X7 | `rexroth-arch01-kernel` / `rexroth-arch02-kernel` | 6.12.40-ctrlx-59.2 / 59.3 |
| Version Guard | `rexroth-version-guard` | 4.6.0 |

Get the system apps from the ctrlX Store or the Bosch Rexroth download area for your ctrlX OS version.
Install all system apps together, or in this order: snapd, core24, Device Admin, then the other apps (ctrlX OS release notes).

## Standard ctrlX apps

These apps are installed on every b»controlled box and kept in the App Storage of the device.

| App | Snap name | Version | Use |
|---|---|---|---|
| EtherCAT Master | `rexroth-ethercatmaster` | 4.6.0 | EtherCAT robots and I/O |
| Firewall | `rexroth-firewall` | 4.6.4 | Engineering access to the robot, see [SETUP_ENGINEERING_ACCESS.md](SETUP_ENGINEERING_ACCESS.md) |
| IXON Cloud Connector | `ctrlx-ixon-ixagent` | 0.1.21 | Remote support |
| Base 22 | `core22` | 20260410 | Required by the IXON agent |

## b»controlled box apps

| App | Snap name | Version | Required |
|---|---|---|---|
| ROS 2 Jazzy runtime | `ros2-base-jazzy` | 3.1.0 | Yes |
| Zenoh router | `zenohd-router-jazzy-binary` | 1.0 | Yes; b»controlled box uses Zenoh, see [SETUP_ZENOH.md](SETUP_ZENOH.md) |
| b»controlled box RT | `b-controlled-box-rt-jazzy` | project build | Yes |
| b»controlled box UI | `b-controlled-box-ui-jazzy` | 1.3.0 (newest build) | Yes |
| Robot-specific apps | e.g. `ur-tool-communication-node`, `b-controlled-box-franka-gripper` | per robot | Only for that robot |

These apps are not signed by Bosch Rexroth.
Allow "installation from unknown sources" before you install them.

## Download

You get the b»controlled box apps from b»robotized as download links for your project.
The RT app is built for your project and robot; its version contains the project name, for example `b-controlled-box-rt-jazzy_1.7.2.-4.x-20-<project>_arm64.snap`.

| App | arm64 | amd64 |
|---|---|---|
| `ros2-base-jazzy` 3.1.0 | link from b»robotized | link from b»robotized |
| `zenohd-router-jazzy-binary` 1.0 | link from b»robotized | link from b»robotized |
| `b-controlled-box-ui-jazzy` 1.3.0 | link from b»robotized | link from b»robotized |
