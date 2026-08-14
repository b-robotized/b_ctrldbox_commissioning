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

The `b_ctrldbox_rsi_eth.xml` (IP/port/element mapping) is identical across all
four RSI versions for a given configuration (RSI 6.x's ethernet config was
confirmed byte-identical to the others, mod line endings), so it lives once
under `common/` instead of being duplicated in each `rsi_*.x/` folder. Only
the RSI Visual project itself (`.rsix` or `.rsi`/`.rsi.diagram`/`.rsi.xml`)
differs between RSI versions.

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
| KSS 8.3, 8.4 | RSI 3.3.x | *(not recorded)* | ⬜ Not yet tested |
| KSS 9.2.2 (iiQKA.OS2) | RSI 6.2.1.4 | KUKA.EthernetKRL 6.1.2.12 | ✅ Tested on real controller — config in `rsi_6.x/`, import steps in "iiQKA.OS2 (RSI 6.x) Deployment" below |

Update this table with the exact patch versions once validated — don't widen
a row to a whole KSS range until every version in that range has actually
been tested.

## Overview

The b_ctrldbox RSI setup supports three different configurations:

1. **Standard** - Basic 6-axis robot control
2. **External Axis** - 6-axis robot + external axes (e.g., linear rails, positioners)
3. **GPIO** - 6-axis robot + GPIO digital I/O support

## Configuration Details

### 1. Standard Configuration

**Location:** `Config/User/Common/SensorInterface/rsi_4.1.x/` (KSS 8.6) or
`Config/User/Common/SensorInterface/rsi_4.0.x/` (KSS 8.5) or
`Config/User/Common/SensorInterface/rsi_3.3.x/` (KSS 8.3, 8.4) or
`Config/User/Common/SensorInterface/rsi_6.x/` (KSS 9.2.2, iiQKA.OS2 —
imported via iiQWorks.Sim, not `deploy.bat`), plus the shared
`Config/User/Common/SensorInterface/common/`

**Files:**
- `common/b_ctrldbox_rsi_eth.xml` - Ethernet configuration (same for all RSI versions, including 6.x)
- RSI 4.0.x / 4.1.x / 6.x (KSS 8.5 / 8.6 / 9.2.2): `b_ctrldbox_rsi.rsix` - packed RSI Visual project
- RSI 3.3.x (KSS 8.3, 8.4): `b_ctrldbox_rsi.rsi` + `.rsi.diagram` + `.rsi.xml` - split RSI Visual project

RSI 6.x only has the Standard variant ported so far — no `ext_axis`/`gpios`
subfolder yet, see "iiQKA.OS2 (RSI 6.x) Deployment" below.

**Features:**
- ✅ 6 robot axes (A1-A6)
- ✅ Stop signal
- ✅ Standard RSI corrections

**RECEIVE Elements (XML):**
```xml
Index 1: Stop (BOOL)
Index 2-7: AK.A1 - AK.A6 (robot joint corrections)
```

**SEND Elements (XML):**
```xml
DEF_RIst   - Cartesian position (actual)
DEF_AIPos  - Joint position (actual)
DEF_EIPos  - External axis position (always 0 for standard)
DEF_Delay  - Late packet counter
```

**Use Cases:**
- Standard 6-axis robot applications
- No external axes required
- Simple RSI control

---

### 2. External Axis Configuration

**Location:** `Config/User/Common/SensorInterface/rsi_4.1.x/ext_axis/` (KSS 8.6) or
`Config/User/Common/SensorInterface/rsi_4.0.x/ext_axis/` (KSS 8.5) or
`Config/User/Common/SensorInterface/rsi_3.3.x/ext_axis/` (KSS 8.3, 8.4), plus
the shared `Config/User/Common/SensorInterface/common/ext_axis/`

**Files:**
- `common/ext_axis/b_ctrldbox_rsi_eth.xml` - Ethernet configuration with external axis (same for all RSI versions)
- RSI 4.0.x / 4.1.x (KSS 8.5 / 8.6): `b_ctrldbox_rsi.rsix` - packed RSI Visual project with AxisCorrExt
- RSI 3.3.x (KSS 8.3, 8.4): `b_ctrldbox_rsi.rsi` + `.rsi.diagram` + `.rsi.xml` - split RSI Visual project with AxisCorrExt

**Features:**
- ✅ 6 robot axes (A1-A6)
- ✅ Stop signal
- ✅ Up to 6 external axes (E1-E6)
- ✅ AxisCorrExt object for external axis control

**RECEIVE Elements (XML):**
```xml
Index 1: Stop (BOOL)
Index 2-7: AK.A1 - AK.A6 (robot joint corrections)
Index 8: EK.E1 (external axis correction)
```

**SEND Elements (XML):**
```xml
DEF_RIst   - Cartesian position (actual)
DEF_AIPos  - Joint position (actual)
DEF_EIPos  - External axis position (actual E1 position)
DEF_Delay  - Late packet counter
```

**Use Cases:**
- Robot on linear rail (7th axis)
- Robot with positioner (turntable, tilt axis)
- Robot with gantry system
- Multi-robot coordinated motion with external axes

**External Axis Types:**
- **Linear axes:** Track, slide, gantry (units: mm)
- **Rotary axes:** Turntables, positioners (units: degrees)

**Limits (configured in .rsix):**
- E1: ±1000 (linear mm or rotary degrees)
- E2-E6: ±5 (placeholders, can be adjusted)

---

### 3. GPIO Configuration

**Location:** `Config/User/Common/SensorInterface/rsi_4.1.x/gpios/` (KSS 8.6) or
`Config/User/Common/SensorInterface/rsi_4.0.x/gpios/` (KSS 8.5) or
`Config/User/Common/SensorInterface/rsi_3.3.x/gpios/` (KSS 8.3, 8.4), plus
the shared `Config/User/Common/SensorInterface/common/gpios/`

**Files:**
- `common/gpios/b_ctrldbox_rsi_eth.xml` - Ethernet configuration with GPIO (same for all RSI versions)
- RSI 4.0.x / 4.1.x (KSS 8.5 / 8.6): `b_ctrldbox_rsi.rsix` - packed RSI Visual project with GPIO blocks
- RSI 3.3.x (KSS 8.3, 8.4): `b_ctrldbox_rsi.rsi` + `.rsi.diagram` + `.rsi.xml` - split RSI Visual project with GPIO blocks

**Features:**
- ✅ 6 robot axes (A1-A6)
- ✅ Stop signal
- ✅ Digital I/O synchronized with RSI cycle
- ✅ GPIO.01, GPIO.02 for real-time I/O

**RECEIVE Elements (XML):**
```xml
Index 1: Stop (BOOL)
Index 2-7: AK.A1 - AK.A6 (robot joint corrections)
Index 8: GPIO.01 (digital input from external controller)
```

**SEND Elements (XML):**
```xml
DEF_RIst   - Cartesian position (actual)
DEF_AIPos  - Joint position (actual)
DEF_EIPos  - External axis position (with GPIO info)
DEF_Delay  - Late packet counter
GPIO.01    - Digital output to external controller
GPIO.02    - Digital output to external controller
```

**Use Cases:**
- Synchronized trigger signals (welding, gripper, sensors)
- Real-time I/O during motion
- Sensor-guided applications requiring fast digital feedback
- Multi-robot coordination with handshake signals

**GPIO Capabilities:**
- Bidirectional digital I/O
- Synchronized with RSI cycle (4ms or 12ms)
- Up to 2 GPIO channels (expandable)

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
| **External Axes** | ❌ | ✅ E1-E6 | ❌ |
| **GPIO** | ❌ | ❌ | ✅ 2 channels |
| **RECEIVE Indices** | 1-7 (Stop + A1-A6) | 1-8 (Stop + A1-A6 + E1) | 1-8 (Stop + A1-A6 + GPIO.01) |
| **Monitor Channels** | 6 | 12 | 6+ |
| **Use Case** | Standard robot | Robot + rail/positioner | Robot + synchronized I/O |

---

**Version:** Updated for kuka-external-control-sdk compatibility
**Last Updated:** February 2026
