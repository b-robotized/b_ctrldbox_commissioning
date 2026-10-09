# RSI Configuration Options

This document describes the different RSI configuration options available in the b_ctrldbox deployment.

## RSI / KSS Version Mapping

The RSI Visual packaging format and object graph differ by RSI version, which
in turn is tied to the KSS version on the controller:

| RSI version | KSS version | Packaging format |
|---|---|---|
| RSI 3.3.x | KSS 8.3, 8.4 | Split project: `.rsi` + `.rsi.diagram` + `.rsi.xml` |
| RSI 4.0.x | KSS 8.5 | Packed project: single `.rsix` file |
| RSI 4.1.x | KSS 8.6 | Packed project: single `.rsix` file |
| RSI 6.2.x | KSS 9.2.2 (iiQKA.OS2) | Packed project (`.rsix`), lives in `rsi_6.x/` like the others, but **imported via iiQWorks.Sim, not `deploy.bat`** — see "iiQKA.OS2 (RSI 6.x) Deployment" below |

Pick the folder matching your controller's KSS version, not just "packed vs.
split" — RSI 4.0.x, 4.1.x, and 6.x all use `.rsix` but are not interchangeable.

The `b_ctrldbox_rsi_eth.xml` (IP/port/element mapping) lives once per
configuration under `common/` and is shared by all RSI versions; only the RSI
Visual project itself (`.rsix` or `.rsi`/`.rsi.diagram`/`.rsi.xml`) differs
between RSI versions.

> ⚠️ **Only the RSI 4.1.x contexts match the current ethernet configs.** The
> configurations now send joint torques, motor currents, program status and setpoint
> positions, which need the `GearTorque`, `Status` and `OV_PRO` objects (and
> `GearTorqueExt`/`AxisCorrExt` for the external axis) in the context. The RSI 3.3.x,
> 4.0.x and 6.x contexts have not been updated yet - they don't contain these objects,
> so deploying them together with the current `common/` files gives a mismatching setup
> (`deploy.bat` warns about this). Port the 4.1.x objects to them before using them.

**KSS 9.2.2 (iiQKA.OS2) is a different platform with a different deployment
mechanism, even though its config files now live in this same folder tree.**
Classic KSS (8.3–8.6, above) gives RSI its own virtual network interface,
separate from EKI, and deploys via `deploy.bat`. On KSS 9.2.2 / iiQKA.OS2, RSI
does not get a virtual interface — **EKI and RSI use the same IP address** on
the same network/interface as the rest of the controller — and there is no
`deploy.bat` equivalent; files are imported one by one (or folder by folder)
through iiQWorks.Sim. Don't assume separate addresses per protocol, and don't
expect `deploy.bat` to handle `rsi_6.x/` or the `EthernetKRL/iiqka_os2/`
config. See "iiQKA.OS2 (RSI 6.x) Deployment" below for the import steps.

### Tested Combinations

The mapping above says which RSI packaging is *compatible* with which KSS
range. It does not mean every version in that range has been run against
real hardware — only these specific patch versions have been validated so
far:

| KSS version | RSI version | EthernetKRL version | Status |
|---|---|---|---|
| KSS 8.5.5 | RSI 4.0.6 | KUKA.EthernetKRL 3.1.2 | ✅ Tested on real controller |
| KSS 8.6.5 | RSI 4.1.6 | *(not recorded)* | ✅ Tested on real controller |
| KSS 8.6.8 | RSI 4.1.3 | *(not recorded)* | ✅ Tested on real controller (KR 210 R3100-2, `rsi_only`): Standard configuration (torques, currents, status, Cartesian pose). Setpoint positions (`DEF_ASPos`/`DEF_ESPos`) tested with a simulated controller only. External Axis: driver side tested without a physical axis. GPIO part ⬜ not yet tested |
| KSS 8.3, 8.4 | RSI 3.3.x | *(not recorded)* | ⬜ Not yet tested |
| KSS 9.2.2 (iiQKA.OS2) | RSI 6.2.1.4 | KUKA.EthernetKRL 6.1.2.12 | ✅ Tested on real controller — config in `rsi_6.x/`, import steps in "iiQKA.OS2 (RSI 6.x) Deployment" below |

Update this table with the exact patch versions once validated — don't widen
a row to a whole KSS range until every version in that range has actually
been tested.

## Overview

The b_ctrldbox RSI setup supports three configurations:

1. **Standard** - 6-axis robot with joint torques, motor currents, program status/speed
   scaling, actual and setpoint Cartesian pose and axis-specific setpoint positions
2. **External Axis** - Standard + one external axis (E1), e.g. a KUKA linear unit (KL)
3. **GPIO** - Standard + 8 digital inputs / 12 digital outputs

All three use XML elements the driver only reads with the **driver YAML**
`workspaces/kuka/rsi_xml_config/b_ctrldbox_rsi_xml_config.yaml` (`rsi_xml_config_file:=...`). The Standard configuration is
active as shipped; uncomment the lines tagged `[EXT_AXIS]` or `[GPIO]` for the other two. The YAML is the single
source of truth: the ethernet config of each configuration is generated from it, so the
driver and the controller always agree on the message layout:

```bash
ros2 run kuka_rsi_driver generate_krc_rsi_config.py \
  --config workspaces/kuka/rsi_xml_config/b_ctrldbox_rsi_xml_config.yaml \
  --client-ip 10.23.23.28 --client-port 28283 \
  --output workspaces/kuka/kss_deployment/Config/User/Common/SensorInterface/common/[ext_axis/|gpios/]b_ctrldbox_rsi_eth.xml
```

Elements without an index (`DEF_*`, `INDX="INTERNAL"`) are KRC built-ins. Elements with an
index must be wired to the matching `Ethernet_1` port in the `.rsix` - the shipped RSI 4.1.x
contexts already contain these objects. Requires a `kuka_rsi_driver` version with setpoint
position support (`position_setpoint` state interface).

## Configuration Details

### 1. Standard Configuration

**Location:** `Config/User/Common/SensorInterface/rsi_4.1.x/` (KSS 8.6), plus the shared
`Config/User/Common/SensorInterface/common/`

**Files:**
- `common/b_ctrldbox_rsi_eth.xml` - Ethernet configuration, **generated** from `b_ctrldbox_rsi_xml_config.yaml` (as shipped)
- `rsi_4.1.x/b_ctrldbox_rsi.rsix` - packed RSI Visual project with `GearTorque`, `Status` and `OV_PRO` objects
- `workspaces/kuka/rsi_xml_config/b_ctrldbox_rsi_xml_config.yaml` - driver-side RSI XML config for all three configurations (**not** deployed to the controller)

**Features:**
- ✅ 6 robot axes (A1-A6) and Stop signal
- ✅ Joint torques (`GearTorque`, measured, gear side) → joint `effort` state interfaces
- ✅ Motor currents (`DEF_MACur`) → joint `current` state interfaces
- ✅ Axis-specific setpoint positions (`DEF_ASPos`) → joint `position_setpoint` state interfaces
- ✅ Program state (`$PRO_STATE1`) and program override (`$OV_PRO`) → `robot_status` sensor
- ✅ Actual (`DEF_RIst`) and setpoint (`DEF_RSol`) Cartesian pose → `cartesian_pose` / `cartesian_setpoint` sensors

**RECEIVE Elements (XML):**
```xml
Index 1: Stop (BOOL)
Index 2-7: AK.A1 - AK.A6 (robot joint corrections)
```

**SEND Elements (XML):**
```xml
DEF_RIst       - Cartesian position (actual)
DEF_RSol       - Cartesian position (setpoint)
DEF_AIPos      - Joint position (actual)
Index 1-6:     GearTorque.A1 - GearTorque.A6   (<- GearTorque_1, TorqueSource=Measured, LocationOnJoint=Gear)
DEF_MACur      - Motor currents A1-A6
DEF_ASPos      - Axis-specific setpoint positions A1-A6
Index 7:       ProgStatus.R (LONG)             (<- Status_1, Type=ProState_R)
Index 8:       OvPro.R                         (<- OV_PRO_1)
DEF_Delay      - Late packet counter
```

**Driver:** `rsi_xml_config_file:=<abs. path>/b_ctrldbox_rsi_xml_config.yaml` (as shipped), plus
`read_robot_status:=true read_cartesian_pose:=true read_cartesian_setpoint:=true` for the
status and pose sensors. Joint torques, currents and setpoint positions need no extra flag.

**Notes from testing (KSS 8.6.8 / RSI 4.1.3):**
- Positions of both Cartesian poses are published in metres (driver converts from KUKA mm).
- The Cartesian pose is the TCP of the **active `$TOOL` in the active `$BASE`**, not the flange.
  Select tool 0 / base 0 in the RSI program if it should match `base_link` → `tool0`.
- The setpoint pose (`RSol`) did not follow the motion commanded through RSI corrections - it
  stayed at the pose where `RSI_MOVECORR()` was entered. Check whether `ASPos` behaves the same
  before relying on the setpoint positions as commanded-position feedback.

---

### 2. External Axis Configuration

**Location:** `Config/User/Common/SensorInterface/rsi_4.1.x/ext_axis/` (KSS 8.6), plus the
shared `Config/User/Common/SensorInterface/common/ext_axis/`

**Files:**
- `common/ext_axis/b_ctrldbox_rsi_eth.xml` - Ethernet configuration, **generated** from `b_ctrldbox_rsi_xml_config.yaml` with the `[EXT_AXIS]` lines uncommented
- `rsi_4.1.x/ext_axis/b_ctrldbox_rsi.rsix` - Standard project plus `GearTorqueExt` and `AxisCorrExt` objects
- `workspaces/kuka/rsi_xml_config/b_ctrldbox_rsi_xml_config.yaml` - uncomment the lines tagged `[EXT_AXIS]`

**Features:**
- ✅ Everything from the Standard configuration
- ✅ External axis E1: position (`DEF_EIPos`), correction (`EK.E1` → `AxisCorrExt_1`, limits ±1000 mm or deg),
  torque (`GearTorqueExt`), motor current (`DEF_MECur`) and setpoint position (`DEF_ESPos`)

**RECEIVE Elements (XML):**
```xml
Index 1: Stop (BOOL)
Index 2-7: AK.A1 - AK.A6 (robot joint corrections)
Index 8: EK.E1 (external axis correction, -> AxisCorrExt_1)
```

**SEND Elements (XML):**
```xml
DEF_RIst, DEF_RSol, DEF_AIPos, DEF_EIPos
Index 1-6:   GearTorque.A1 - GearTorque.A6
Index 7:     GearTorqueExt.E1                  (<- GearTorqueExt_1)
DEF_MACur, DEF_MECur, DEF_ASPos, DEF_ESPos
Index 8:     ProgStatus.R (LONG)
Index 9:     OvPro.R
DEF_Delay
```

**Driver:** as Standard with the `[EXT_AXIS]` lines of the YAML uncommented, plus
`use_external_axis:=true kl_model:=kl100_2 kl_prefix:=rail_`. The YAML refers to the external
joint as `rail_joint_1`; adapt `joint_identifier` if you change `kl_prefix`.

**Use Cases:** robot on a linear unit (7th axis), positioner, gantry.

---

### 3. GPIO Configuration

**Location:** `Config/User/Common/SensorInterface/rsi_4.1.x/gpios/` (KSS 8.6), plus the shared
`Config/User/Common/SensorInterface/common/gpios/`

**Files:**
- `common/gpios/b_ctrldbox_rsi_eth.xml` - Ethernet configuration, **generated** from `b_ctrldbox_rsi_xml_config.yaml` with the `[GPIO]` lines uncommented
- `rsi_4.1.x/gpios/b_ctrldbox_rsi.rsix` - Standard project plus `DigIn`/`Map2DigOut` objects
- `workspaces/kuka/rsi_xml_config/b_ctrldbox_rsi_xml_config.yaml` - uncomment the lines tagged `[GPIO]`

**Features:**
- ✅ Everything from the Standard configuration
- ✅ 8 digital inputs (`DigIn` → `$IN[132]`..`$IN[139]`) and 12 digital outputs (`Map2DigOut` → `$OUT[33]`..`$OUT[36]`, `$OUT[39]`..`$OUT[46]`), synchronized with the RSI cycle

The signal names (`input_01`..`input_08`, `output_01`..`output_12`) are generic placeholders
and the robot I/O numbers are an example mapping. Adapt both to your application in RSI
Visual, in the YAML (then regenerate the ethernet config) and in the robot description's GPIO
interfaces (`gpio_config.xacro`) - see `LAUNCH.md`, "Adding or renaming a GPIO".

**RECEIVE Elements (XML):**
```xml
Index 1: Stop (BOOL)
Index 2-7: AK.A1 - AK.A6 (robot joint corrections)
Index 8-11: GPIO.output_01 - GPIO.output_04 (-> $OUT[33]..$OUT[36])
Index 12-19: GPIO.output_05 - GPIO.output_12 (-> $OUT[39]..$OUT[46])
```

**SEND Elements (XML):**
```xml
DEF_RIst, DEF_RSol, DEF_AIPos
Index 1-6:   GearTorque.A1 - GearTorque.A6
DEF_MACur, DEF_ASPos
Index 7-14:  GPIO.input_01 - GPIO.input_08  (<- $IN[132]..$IN[139])
Index 15:    ProgStatus.R (LONG)
Index 16:    OvPro.R
DEF_Delay
```

⚠️ The indices of `ProgStatus.R`/`OvPro.R` differ from the Standard configuration (15/16
instead of 7/8) because the driver emits GPIO fields before the robot status fields. Always
use the `.rsix` and ethernet config from the **same** configuration folder.

**Driver:** as Standard with the `[GPIO]` lines of the YAML uncommented, plus `use_gpio:=true`. The robot description must
declare exactly these 8 GPIO state and 12 GPIO command interfaces - the driver refuses to start
if the number of GPIO states doesn't match the YAML. RSI writes the mapped outputs every cycle
while RSI is active; don't write the same `$OUT` from KRL at the same time.

**Status:** ⬜ the GPIO part is not yet tested on a real controller.

---

## Deployment

### Using deploy.bat

When you run `deploy.bat`, you'll first be prompted to select the RSI version
(matching your controller's KSS version), then the RSI configuration:

```
============================================
KUKA b_ctrldbox Deployment
============================================

Select RSI version (matches your KSS version):
  1. RSI 3.3.x - KSS 8.3, 8.4 (separate .rsi / .rsi.diagram / .rsi.xml files)
  2. RSI 4.0.x - KSS 8.5 (single .rsix file)
  3. RSI 4.1.x - KSS 8.6 (single .rsix file)

Enter selection (1/2/3):

Select RSI configuration:
  1. Standard (6 robot axes only)
  2. External Axis (6 robot axes + external axes support)
  3. GPIO (6 robot axes + GPIO support)

Enter selection (1/2/3):
```

If an RSI version other than 4.1.x is selected, `deploy.bat` warns that its context doesn't
match the shared ethernet config and asks for confirmation. At the end it prints which driver
YAML to use (`b_ctrldbox_rsi_xml_config.yaml`, and which tagged lines to uncomment).

The RSI version determines *how* the RSI context is packaged (RSI 4.0.x and
4.1.x use a single `.rsix` file; RSI 3.3.x requires the RSIVisual project
split into `.rsi` / `.rsi.diagram` / `.rsi.xml`) - the RSI/EKI logic itself is
identical across all three.

### What Gets Deployed

Based on your selections:

| RSI version | KSS version | RSI selection | Version-specific Source Folder | Version-specific Files | Shared Source Folder | Shared File |
|---|---|---|---|---|---|---|
| **RSI 3.3.x** | 8.3, 8.4 | 1 (Standard) | `.../rsi_3.3.x/` | `b_ctrldbox_rsi.rsi` + `.rsi.diagram` + `.rsi.xml` | `.../common/` | `b_ctrldbox_rsi_eth.xml` |
| **RSI 3.3.x** | 8.3, 8.4 | 2 (External Axis) | `.../rsi_3.3.x/ext_axis/` | External axis versions | `.../common/ext_axis/` | `b_ctrldbox_rsi_eth.xml` |
| **RSI 3.3.x** | 8.3, 8.4 | 3 (GPIO) | `.../rsi_3.3.x/gpios/` | GPIO versions | `.../common/gpios/` | `b_ctrldbox_rsi_eth.xml` |
| **RSI 4.0.x** | 8.5 | 1 (Standard) | `.../rsi_4.0.x/` | `b_ctrldbox_rsi.rsix` | `.../common/` | `b_ctrldbox_rsi_eth.xml` |
| **RSI 4.0.x** | 8.5 | 2 (External Axis) | `.../rsi_4.0.x/ext_axis/` | External axis versions | `.../common/ext_axis/` | `b_ctrldbox_rsi_eth.xml` |
| **RSI 4.0.x** | 8.5 | 3 (GPIO) | `.../rsi_4.0.x/gpios/` | GPIO versions | `.../common/gpios/` | `b_ctrldbox_rsi_eth.xml` |
| **RSI 4.1.x** | 8.6 | 1 (Standard) | `.../rsi_4.1.x/` | `b_ctrldbox_rsi.rsix` | `.../common/` | `b_ctrldbox_rsi_eth.xml` |
| **RSI 4.1.x** | 8.6 | 2 (External Axis) | `.../rsi_4.1.x/ext_axis/` | External axis versions | `.../common/ext_axis/` | `b_ctrldbox_rsi_eth.xml` |
| **RSI 4.1.x** | 8.6 | 3 (GPIO) | `.../rsi_4.1.x/gpios/` | GPIO versions | `.../common/gpios/` | `b_ctrldbox_rsi_eth.xml` |

Only the RSI 4.1.x rows match the current shared ethernet configs (see the note at the top).

(`.../` = `Config/User/Common/SensorInterface/`. Both the version-specific and
shared source folders are copied into the same destination on the
controller, `C:\KRC\ROBOTER\Config\User\Common\SensorInterface\`.)

All configurations also deploy the same shared files regardless of RSI/KSS version:
- RSI ethernet config (from `Config/User/Common/SensorInterface/common/`)
- RSI program files (from `KRC/R1/Program/RSI_kss/`)
- EKI config files (from `Config/User/Common/EthernetKRL/kss/`)
- EKI server programs (from `KRC/R1/Program/EKIServer_kss/`)

This applies to `deploy.bat` (classic KSS: RSI 3.3.x/4.0.x/4.1.x). iiQKA.OS2
(RSI 6.x) has its own EKI config folder, `Config/User/Common/EthernetKRL/iiqka_os2/`,
and its own program folders, `KRC/R1/Program/RSI_6.x/` and
`KRC/R1/Program/EKIServer_6.x/` — same tree as the classic-KSS ones, kept as
separate version-suffixed folders because the KRL/EKI implementation itself
differs per platform (confirmed by diff, not just renamed). These are
imported via iiQWorks.Sim, not `deploy.bat` — see "iiQKA.OS2 (RSI 6.x)
Deployment" below.

### Versions on record

Folder names throughout this document use a version suffix where one is
known, and a platform name (`kss`) where it isn't — a placeholder, not a
claim that no version exists. Fill in the blank cell once known, and rename
`EKIServer_kss`/`EthernetKRL/kss` to match.

| Component | KSS 8.3, 8.4 | KSS 8.5 | KSS 8.6 | KSS 9.2.2 (iiQKA.OS2) |
|---|---|---|---|---|
| RSI | 3.3.x | 4.0.6 | 4.1.6 | 6.2.1.4 |
| KUKA.EthernetKRL | *(not recorded)* | 3.1.2 (KSS 8.5.5) | *(not recorded)* | 6.1.2.12 |

| Folder | Package version | Source |
|---|---|---|
| `RSI_kss`, `SensorInterface/rsi_3.3.x`/`4.0.x`/`4.1.x` | RSI 3.3.x / 4.0.x / 4.1.x | tracked per-KSS-version above, format changed across releases |
| `RSI_6.x`, `SensorInterface/rsi_6.x` | RSI 6.2.1.4 | [[maurob-kuka-versions]] |
| `EKIServer_6.x`, `EthernetKRL/iiqka_os2` | KUKA.EthernetKRL 6.1.2.12 | [[maurob-kuka-versions]] |
| `EKIServer_kss`, `EthernetKRL/kss` | KUKA.EthernetKRL 3.1.2 confirmed for the KSS 8.5.5 tested combination — **not confirmed for KSS 8.3/8.4/8.6**, don't assume it's the same across the whole classic-KSS range | [[kuka-rsi-tested-versions]] |

---

## iiQKA.OS2 (RSI 6.x) Deployment

The b»controlled box RSI/EKI setup for controllers running iiQKA.OS2 instead
of classic KSS/VxWorks (no KLI interface, no WorkVisual/USB file transfer).
Config and Program files live in this same repo tree (see the tables above),
but the deployment *mechanism* is genuinely different — no `deploy.bat`
equivalent, manual import through iiQWorks.Sim instead.

Source template: `kuka_external_control_sdk/krc_setup/iiqka_os2/` in
[kroshu/kuka-external-control-sdk](https://github.com/kroshu/kuka-external-control-sdk),
following `doc/iiqka_os2_setup.md`.

### What was changed vs. the upstream template

Comparing `kss_deployment/` to its own upstream template (`krc_setup/kss/`)
showed the b»controlled box customization is only ever: network config
(IP/port) + renaming the RSI context to `b_ctrldbox_rsi` — no KRL program
logic was changed. The same two changes are applied here:

1. **Ethernet config** — originally its own copied file with `IP_NUMBER` set
   to `10.23.23.28` and `PORT` to `28283`. Confirmed byte-identical (mod line
   endings) to `common/b_ctrldbox_rsi_eth.xml`, so the separate copy was
   deleted — RSI 6.x reuses that same shared file, same as RSI 4.0.x/4.1.x.
   Element mapping (Stop, AK.A1-A6, DEF_RIst, DEF_AIPos, DEF_EIPos,
   DEF_Delay) is unchanged from upstream.

2. **`rsi_joint_pos.dat`** (in `RSI_6.x/`) — `CONTEXT_NAME[]` changed from
   `"rsi_joint_pos"` to `"b_ctrldbox_rsi"`, matching the renamed Context file
   below. `rsi_joint_pos.src` itself is untouched — it loads the context via
   the `CONTEXT_NAME[]` variable, not a hardcoded string, so no edit was
   needed there (unlike the classic KSS `.src`, which hardcodes the name in
   `RSI_CREATE(...)`).

3. **`b_ctrldbox_rsi.rsix`** (in `SensorInterface/rsi_6.x/`) — renamed from
   `rsi_joint_pos.rsix` to match `CONTEXT_NAME[]` above. This file is plain
   XML (not binary, despite the extension), and it internally references its
   ethernet config by filename in two places (`ConfigFile` parameter). Both
   were updated from `rsi_ethernet.xml` to `b_ctrldbox_rsi_eth.xml` to
   match — confirmed against the same pattern in the classic KSS `.rsix`,
   which references `b_ctrldbox_rsi_eth.xml` the same way. No other content
   was changed.

Everything else (the EKI interface config in `EthernetKRL/iiqka_os2/`, plus
all of `EKIServer_6.x/` and `RSI_6.x/rsi_helper.*`) is an **unmodified copy**
of the upstream `iiqka_os2` template — verified byte-identical. The EKI
config already used port `54600`, matching `kss_deployment`'s default, so no
change was needed there. Its schema (`<Channel>`) is genuinely different
from classic KSS's EKI schema (`<ETHERNETKRL>`) — confirmed by diffing
both — so unlike the RSI ethernet config, this file could **not** be merged
into a single shared file; it lives as a platform-specific sibling instead.
Same reasoning for the Program folders — diffed every same-named file
against `RSI_kss/`/`EKIServer_kss/` and all differ (different KRL/EKI API
per platform), so `RSI_6.x/`/`EKIServer_6.x/` are version-suffixed siblings,
not shared files.

### Not yet verified

- **Whether renaming the `.rsix` file is sufficient**, or whether iiQWorks.Sim
  also needs the Context explicitly named/registered as `b_ctrldbox_rsi`
  during import (`Option packages > iiQKA.RobotSensorInterface > Context`).
  Classic KSS RSI loads contexts by filename on the controller's filesystem;
  iiQKA.OS2 imports contexts as project artifacts through iiQWorks.Sim, and
  it's not confirmed the naming works the same way. Check this when
  importing.
- **Deliberately deferred:** only the **Standard** configuration (6 axes, no
  external axis, no GPIO) is ported, matching MauRob's current needs. The
  upstream `iiqka_os2` template has External Axis/GPIO equivalents
  (`rsi_ext_axis_ethernet.xml`, `rsi_ext_axis_example.rsix`,
  `rsi_gpio_ethernet.xml`, `rsi_gpio_joint_pos.rsix`) that can be ported the
  same way once Standard is confirmed working on the real controller — don't
  port them speculatively before that.
- **Tested on a real controller** — KSS 9.2.2, RSI 6.2.1.4. See "Tested
  Combinations" above.
- `deploy.bat` (raw file-copy deployment via `xcopy` to Windows filesystem
  paths like `C:\KRC\ROBOTER\...`) does **not** apply here — iiQKA.OS2 has
  no such filesystem, it deploys via iiQWorks.Sim project import (see
  "Import steps" below). No equivalent script is needed unless iiQWorks.Sim
  turns out to support/require its own scripted import.
- **`b_ctrldbox` subfolder not yet mirrored.** `deploy.bat` shows classic KSS
  deploys RSI *and* EKI program files into a dedicated subfolder,
  `C:\KRC\ROBOTER\KRC\R1\Program\b_ctrldbox\` (matching the
  `&PARAM DISKPATH = KRC:\R1\Program\b_ctrldbox` label in those `.src`
  files) — program names are still resolved globally by KRL regardless of
  subfolder, so this is purely organizational, not required for the program
  to run. The iiQKA.OS2 upstream doc (`iiqka_os2_setup.md`) just says
  "import into `KRC/R1/Program`" without specifying a subfolder. "Import
  steps" below currently follows the vendor doc as-is (flat, no subfolder) —
  check in iiQWorks.Sim whether you can name/create a `b_ctrldbox` subfolder
  on import, and do so for consistency with classic KSS if possible.

### Import steps (per `iiqka_os2_setup.md`)

iiQWorks.Sim accepts a whole folder as one import, not just single files.
Import each folder below directly instead of picking files one by one — each
folder contains exactly the one file that belongs at its target location (no
ext_axis/gpio files to accidentally pull in, since only Standard is ported).
This is manual import either way, so unifying the folder structure into this
same repo doesn't change how you import, only where the source files live.

1. Import the `Config/User/Common/SensorInterface/rsi_6.x/` folder (contains
   `b_ctrldbox_rsi.rsix`) under **Option packages >
   iiQKA.RobotSensorInterface > Context**.
2. Import the `Config/User/Common/SensorInterface/common/` folder (contains
   `b_ctrldbox_rsi_eth.xml`, shared with RSI 4.0.x/4.1.x) under the same
   option package's **Ethernet configurations**.

   ⚠️ The shared `common/b_ctrldbox_rsi_eth.xml` now uses the extended message
   layout (torques, status, ...), which the RSI 6.x context does not support yet.
   Until the 6.x context is updated, use the basic ethernet config from the git
   history of this file (before the extended layout) for iiQKA.OS2.
3. Import the `Config/User/Common/EthernetKRL/iiqka_os2/` folder (contains
   `b_ctrldbox_EkiKSSinterface.xml`) under **Option packages >
   iiQKA.EthernetKRL > Context**.
4. Import the `KRC/R1/Program/RSI_6.x/` and `KRC/R1/Program/EKIServer_6.x/`
   folders (version-suffixed siblings of `RSI_kss/`/`EKIServer_kss/` —
   genuinely different KRL/EKI implementation per platform, not unified)
   into `KRC/R1/Program` (into a `b_ctrldbox` subfolder if iiQWorks.Sim
   supports it, matching classic KSS — not yet confirmed either way, see
   above).
5. Deploy the project onto the controller.

---

## Network Configuration

All configurations use the same network settings (defined in XML files):

- **IP Address:** `10.23.23.28`
- **Port:** `28283`
- **Protocol:** UDP
- **SENTYPE:** `KROSHU`

These are consistent across all RSI configurations.

---

## Switching Between Configurations

### Method 1: Re-deploy with deploy.bat

Simply run `deploy.bat` again and select a different configuration. The script will copy the appropriate files to the robot.

### Method 2: Manual File Copy

If you need to switch manually, pick `rsi_3.3.x`, `rsi_4.0.x`, or `rsi_4.1.x`
to match your controller's KSS version (8.3/8.4, 8.5, or 8.6 respectively).
Each switch needs **two** copies: the shared `common\...\b_ctrldbox_rsi_eth.xml`
plus the RSI-version-specific project file(s) — both land in the same
destination folder.

**For External Axis (RSI 4.0.x / KSS 8.5):**
```batch
copy Config\User\Common\SensorInterface\common\ext_axis\b_ctrldbox_rsi_eth.xml C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
copy Config\User\Common\SensorInterface\rsi_4.0.x\ext_axis\b_ctrldbox_rsi.rsix C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
```

**For External Axis (RSI 4.1.x / KSS 8.6):**
```batch
copy Config\User\Common\SensorInterface\common\ext_axis\b_ctrldbox_rsi_eth.xml C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
copy Config\User\Common\SensorInterface\rsi_4.1.x\ext_axis\b_ctrldbox_rsi.rsix C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
```

**For External Axis (RSI 3.3.x / KSS 8.3, 8.4):**
```batch
copy Config\User\Common\SensorInterface\common\ext_axis\b_ctrldbox_rsi_eth.xml C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
copy Config\User\Common\SensorInterface\rsi_3.3.x\ext_axis\*.rsi* C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
```

**For GPIO (RSI 4.0.x / KSS 8.5):**
```batch
copy Config\User\Common\SensorInterface\common\gpios\b_ctrldbox_rsi_eth.xml C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
copy Config\User\Common\SensorInterface\rsi_4.0.x\gpios\b_ctrldbox_rsi.rsix C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
```

**For GPIO (RSI 4.1.x / KSS 8.6):**
```batch
copy Config\User\Common\SensorInterface\common\gpios\b_ctrldbox_rsi_eth.xml C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
copy Config\User\Common\SensorInterface\rsi_4.1.x\gpios\b_ctrldbox_rsi.rsix C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
```

**For GPIO (RSI 3.3.x / KSS 8.3, 8.4):**
```batch
copy Config\User\Common\SensorInterface\common\gpios\b_ctrldbox_rsi_eth.xml C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
copy Config\User\Common\SensorInterface\rsi_3.3.x\gpios\*.rsi* C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
```

Remember to uncomment/comment the matching `[EXT_AXIS]` / `[GPIO]` lines in `b_ctrldbox_rsi_xml_config.yaml`
when switching configurations.

**Back to Standard (RSI 4.0.x / KSS 8.5):**
```batch
copy Config\User\Common\SensorInterface\common\b_ctrldbox_rsi_eth.xml C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
copy Config\User\Common\SensorInterface\rsi_4.0.x\b_ctrldbox_rsi.rsix C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
```

**Back to Standard (RSI 4.1.x / KSS 8.6):**
```batch
copy Config\User\Common\SensorInterface\common\b_ctrldbox_rsi_eth.xml C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
copy Config\User\Common\SensorInterface\rsi_4.1.x\b_ctrldbox_rsi.rsix C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
```

**Back to Standard (RSI 3.3.x / KSS 8.3, 8.4):**
```batch
copy Config\User\Common\SensorInterface\common\b_ctrldbox_rsi_eth.xml C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
copy Config\User\Common\SensorInterface\rsi_3.3.x\b_ctrldbox_rsi.rsi C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
copy Config\User\Common\SensorInterface\rsi_3.3.x\b_ctrldbox_rsi.rsi.diagram C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
copy Config\User\Common\SensorInterface\rsi_3.3.x\b_ctrldbox_rsi.rsi.xml C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
```

---

## Troubleshooting

### Configuration doesn't work after deployment

1. **Verify files copied correctly:**
   ```batch
   dir C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
   ```
   Should show: `b_ctrldbox_rsi_eth.xml` and `b_ctrldbox_rsi.rsix`

2. **Check RSI program references the correct XML:**
   Open the RSI program (e.g., `rsi_joint_pos_4ms.src`) and verify:
   ```krl
   RSI_CREATE("b_ctrldbox_rsi", "b_ctrldbox_rsi.rsix")
   ```

3. **Restart the robot controller** after deployment

### External axis not responding

1. Verify you deployed **External Axis** configuration (option 2)
2. Check that control PC is sending data at RECEIVE index 8 (EK.E1)
3. Verify `DEF_EIPos` is being parsed by control PC
4. Check external axis limits in `.rsix` file

### GPIO signals not working

1. Verify you deployed **GPIO** configuration (option 3)
2. Check control PC is sending/receiving at GPIO indices
3. Verify GPIO wiring in RSI Visual diagram matches expectations
4. Check cycle time (4ms vs 12ms) is appropriate for GPIO speed

---

## Upgrading from Older Versions

If you're upgrading from an older b_ctrldbox version:

### Key Changes

0. **Extended message layout (current version):** all configurations now also send joint
   torques, motor currents, setpoint positions, program status and the setpoint Cartesian pose.
   The former `extended`/`extended_gpios` configurations are now Standard/GPIO. The driver needs
   the matching YAML (`rsi_xml_config_file`), and only the RSI 4.1.x contexts are updated.
1. **Index reordering:** Stop moved from index 7 to index 1, joints A1-A6 now at indices 2-7
2. **DEF_EIPos added:** All configurations now send external axis position (0 for standard config)
3. **SENTYPE updated:** Changed to `KROSHU` for all configurations

### Migration Steps

1. **Update your control PC code** to use new index mapping:
   ```
   OLD: indices 1-6 = A1-A6, index 7 = Stop
   NEW: index 1 = Stop, indices 2-7 = A1-A6
   ```

2. **Parse DEF_EIPos** even if not using external axes (will be 0)

3. **Test thoroughly** with new configuration before production use

---

## References

- [KUKA RSI Documentation](https://www.kuka.com)
- [kuka-external-control-sdk](https://github.com/kroshu/kuka-external-control-sdk)
- b_ctrldbox commissioning repository

---

## Summary Table

| Feature | Standard | External Axis | GPIO |
|---------|----------|---------------|------|
| **Robot Axes** | A1-A6 | A1-A6 | A1-A6 |
| **External Axes** | ❌ | ✅ E1 | ❌ |
| **GPIO** | ❌ | ❌ | ✅ 8 in / 12 out |
| **Torques / currents / setpoint positions / status / Cartesian pose** | ✅ | ✅ (incl. E1) | ✅ |
| **RSI versions matching the shared ethernet config** | 4.1.x | 4.1.x | 4.1.x |
| **Driver YAML (`b_ctrldbox_rsi_xml_config.yaml`)** | as shipped | `[EXT_AXIS]` lines uncommented | `[GPIO]` lines uncommented |
| **RECEIVE Indices** | 1-7 (Stop + A1-A6) | 1-8 (Stop + A1-A6 + E1) | 1-19 (Stop + A1-A6 + 12 GPIO) |
| **Use Case** | Standard robot + load/state monitoring | Robot + rail/positioner | Robot + synchronized I/O |


---

**Version:** Updated for kuka-external-control-sdk compatibility
**Last Updated:** October 2026
