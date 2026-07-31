# RSI Configuration Options

This document describes the different RSI configuration options available in the KSSv8.6 deployment.

## Overview

`deploy.bat` can deploy either of two drivers:

1. **b_ctrldbox** - the ROS 2 EKI+RSI driver in this repo
2. **FZI reference driver** - vendored, unmodified copy of
   [kuka_rsi_driver](https://github.com/fzi-forschungszentrum-informatik/kuka_rsi_driver)
   (see `fzi_reference/README.md`), for side-by-side comparison. RSI-only,
   no EKI command layer, no ext_axis/GPIO variant.

The b_ctrldbox RSI setup supports three different configurations:

1. **Standard** - Basic 6-axis robot control
2. **External Axis** - 6-axis robot + external axes (e.g., linear rails, positioners)
3. **GPIO** - 6-axis robot + GPIO digital I/O support

The FZI reference driver only ships the Standard equivalent, for either
**KR C4** (split `.rsi`/`.rsi.diagram`/`.rsi.xml`) or **KR C5** (packed
`.rsix`) controllers.

## Configuration Details

### 1. Standard Configuration

**Location:** `Config/User/Common/SensorInterface/above_v8.6/` (KSS 8.6+) or
`Config/User/Common/SensorInterface/below_v8.6/` (KSS < 8.6)

**Files:**
- `b_ctrldbox_rsi_eth.xml` - Ethernet configuration
- KSS 8.6+: `b_ctrldbox_rsi.rsix` - packed RSI Visual project
- KSS < 8.6: `b_ctrldbox_rsi.rsi` + `.rsi.diagram` + `.rsi.xml` - split RSI Visual project

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

**Location:** `Config/User/Common/SensorInterface/above_v8.6/ext_axis/` (KSS 8.6+) or
`Config/User/Common/SensorInterface/below_v8.6/ext_axis/` (KSS < 8.6)

**Files:**
- `b_ctrldbox_rsi_eth.xml` - Ethernet configuration with external axis
- KSS 8.6+: `b_ctrldbox_rsi.rsix` - packed RSI Visual project with AxisCorrExt
- KSS < 8.6: `b_ctrldbox_rsi.rsi` + `.rsi.diagram` + `.rsi.xml` - split RSI Visual project with AxisCorrExt

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

**Location:** `Config/User/Common/SensorInterface/above_v8.6/gpios/` (KSS 8.6+) or
`Config/User/Common/SensorInterface/below_v8.6/gpios/` (KSS < 8.6)

**Files:**
- `b_ctrldbox_rsi_eth.xml` - Ethernet configuration with GPIO
- KSS 8.6+: `b_ctrldbox_rsi.rsix` - packed RSI Visual project with GPIO blocks
- KSS < 8.6: `b_ctrldbox_rsi.rsi` + `.rsi.diagram` + `.rsi.xml` - split RSI Visual project with GPIO blocks

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

When you run `deploy.bat`, you'll first be prompted to select the driver,
then (depending on the driver) either the KSS version + RSI configuration,
or the controller generation:

```
============================================
KUKA Deployment
============================================

Select driver to deploy:
  1. b_ctrldbox (ROS 2 EKI+RSI driver)
  2. FZI reference driver (kuka_rsi_driver, RSI-only, no ext_axis/GPIO)

Enter selection (1/2):
```

**If you selected b_ctrldbox:**

```
Select KSS version:
  1. KSS 8.6 and above (single .rsix file)
  2. KSS below 8.6 (separate .rsi / .rsi.diagram / .rsi.xml files)

Enter selection (1/2):

Select RSI configuration:
  1. Standard (6 robot axes only)
  2. External Axis (6 robot axes + external axes support)
  3. GPIO (6 robot axes + GPIO support)

Enter selection (1/2/3):
```

The KSS version determines *how* the RSI context is packaged (KSS 8.6+ uses a
single `.rsix` file; older versions require the RSIVisual project split into
`.rsi` / `.rsi.diagram` / `.rsi.xml`) - the RSI/EKI logic itself is identical
across both.

**If you selected the FZI reference driver:**

```
Select KUKA controller generation:
  1. KR C4 (split .rsi / .rsi.diagram / .rsi.xml files)
  2. KR C5 (single .rsix file)

Enter selection (1/2):
```

### What Gets Deployed

Based on your selections:

| Driver | KSS version / generation | RSI selection | Source Folder | Files Deployed |
|---|---|---|---|---|
| b_ctrldbox | 8.6 and above | 1 (Standard) | `Config/User/Common/SensorInterface/above_v8.6/` | `b_ctrldbox_rsi_eth.xml` + `b_ctrldbox_rsi.rsix` |
| b_ctrldbox | 8.6 and above | 2 (External Axis) | `Config/User/Common/SensorInterface/above_v8.6/ext_axis/` | External axis versions |
| b_ctrldbox | 8.6 and above | 3 (GPIO) | `Config/User/Common/SensorInterface/above_v8.6/gpios/` | GPIO versions |
| b_ctrldbox | below 8.6 | 1 (Standard) | `Config/User/Common/SensorInterface/below_v8.6/` | `b_ctrldbox_rsi_eth.xml` + `b_ctrldbox_rsi.rsi` + `.rsi.diagram` + `.rsi.xml` |
| b_ctrldbox | below 8.6 | 2 (External Axis) | `Config/User/Common/SensorInterface/below_v8.6/ext_axis/` | External axis versions |
| b_ctrldbox | below 8.6 | 3 (GPIO) | `Config/User/Common/SensorInterface/below_v8.6/gpios/` | GPIO versions |
| FZI reference | KR C4 | n/a | `fzi_reference/Config/User/Common/SensorInterface/KR_C4/` | `ros_rsi_ethernet.xml` + `ros_rsi.rsi` + `.rsi.diagram` + `.rsi.xml` |
| FZI reference | KR C5 | n/a | `fzi_reference/Config/User/Common/SensorInterface/KR_C5/` | `ros_rsi_ethernet.xml` + `ros_rsi.rsix` |

All b_ctrldbox configurations also deploy the same shared files regardless of KSS version:
- RSI program files (from `KRC/R1/Program/RSI/`)
- EKI config files (from `Config/User/Common/EthernetKRL/`)
- EKI server programs (from `KRC/R1/Program/EKIserver/`)

The FZI reference driver has no EKI layer - it only deploys its RSI config
files above, plus `kuka_rsi_driver.src` (from
`fzi_reference/KRC/R1/Program/FZIdriver/KR_C4/` or `KR_C5/`) to
`C:\KRC\ROBOTER\KRC\R1\Program\fzi_rsi_driver` - a separate program folder
from `b_ctrldbox`'s, so deploying one driver never touches the other's
program files.

### Stale File Cleanup

`Config\User\Common\SensorInterface` on the controller is a single shared
folder - every b_ctrldbox version/variant and the FZI reference driver write
into it under their own filenames. Because of that, switching what you
deploy (e.g. `above_v8.6` → `below_v8.6`, `Standard` → `ext_axis`, or
b_ctrldbox → FZI) does **not** automatically remove the previous files, since
each combination only overwrites files it actually ships.

To prevent stale/conflicting RSI config files from lingering on the
controller, `deploy.bat` deletes every known filename from that folder before
copying the newly selected set:

```
b_ctrldbox_rsi.rsix, b_ctrldbox_rsi.rsi, b_ctrldbox_rsi.rsi.diagram,
b_ctrldbox_rsi.rsi.xml, b_ctrldbox_rsi_eth.xml,
ros_rsi.rsix, ros_rsi.rsi, ros_rsi.rsi.diagram, ros_rsi.rsi.xml,
ros_rsi_ethernet.xml
```

Only files that actually exist are deleted (missing ones are silently
skipped), and the script prints which ones it removed. Run `backup.bat`
first if you want a copy of what's about to be cleaned up.

---

## Network Configuration

All b_ctrldbox RSI configurations use the same network settings (defined in
the `b_ctrldbox_rsi_eth.xml` files):

- **IP Address:** `10.23.23.28`
- **Port:** `28283`
- **Protocol:** UDP
- **SENTYPE:** `KROSHU`

These are consistent across all b_ctrldbox RSI configurations.

The FZI reference driver's `ros_rsi_ethernet.xml` has been repointed to the
same `IP_NUMBER`/`PORT` as b_ctrldbox (`10.23.23.28:28283`) so both drivers
can be swapped in/out against the same control-PC endpoint without touching
network config. `SENTYPE` is left as the upstream `KukaRsiDriver` value since
it's just a client identifier, not an addressing parameter.

---

## Switching Between Configurations

### Method 1: Re-deploy with deploy.bat

Simply run `deploy.bat` again and select a different configuration. The script will copy the appropriate files to the robot.

### Method 2: Manual File Copy

If you need to switch manually, pick `above_v8.6` or `below_v8.6` to match your
controller's KSS version.

**For External Axis (KSS 8.6+):**
```batch
copy Config\User\Common\SensorInterface\above_v8.6\ext_axis\*.* C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
```

**For External Axis (KSS < 8.6):**
```batch
copy Config\User\Common\SensorInterface\below_v8.6\ext_axis\*.* C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
```

**For GPIO (KSS 8.6+):**
```batch
copy Config\User\Common\SensorInterface\above_v8.6\gpios\*.* C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
```

**For GPIO (KSS < 8.6):**
```batch
copy Config\User\Common\SensorInterface\below_v8.6\gpios\*.* C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
```

**Back to Standard (KSS 8.6+):**
```batch
copy Config\User\Common\SensorInterface\above_v8.6\b_ctrldbox_rsi_eth.xml C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
copy Config\User\Common\SensorInterface\above_v8.6\b_ctrldbox_rsi.rsix C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
```

**Back to Standard (KSS < 8.6):**
```batch
copy Config\User\Common\SensorInterface\below_v8.6\b_ctrldbox_rsi_eth.xml C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
copy Config\User\Common\SensorInterface\below_v8.6\b_ctrldbox_rsi.rsi C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
copy Config\User\Common\SensorInterface\below_v8.6\b_ctrldbox_rsi.rsi.diagram C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
copy Config\User\Common\SensorInterface\below_v8.6\b_ctrldbox_rsi.rsi.xml C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
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
