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

Pick the folder matching your controller's KSS version, not just "packed vs.
split" — RSI 4.0.x and 4.1.x both use `.rsix` but are not interchangeable.

## Overview

The b_ctrldbox RSI setup supports three different configurations:

1. **Standard** - Basic 6-axis robot control
2. **External Axis** - 6-axis robot + external axes (e.g., linear rails, positioners)
3. **GPIO** - 6-axis robot + GPIO digital I/O support

## Configuration Details

### 1. Standard Configuration

**Location:** `Config/User/Common/SensorInterface/rsi_4.1.x/` (KSS 8.6) or
`Config/User/Common/SensorInterface/rsi_4.0.x/` (KSS 8.5) or
`Config/User/Common/SensorInterface/rsi_3.3.x/` (KSS 8.3, 8.4)

**Files:**
- `b_ctrldbox_rsi_eth.xml` - Ethernet configuration
- RSI 4.0.x / 4.1.x (KSS 8.5 / 8.6): `b_ctrldbox_rsi.rsix` - packed RSI Visual project
- RSI 3.3.x (KSS 8.3, 8.4): `b_ctrldbox_rsi.rsi` + `.rsi.diagram` + `.rsi.xml` - split RSI Visual project

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
`Config/User/Common/SensorInterface/rsi_3.3.x/ext_axis/` (KSS 8.3, 8.4)

**Files:**
- `b_ctrldbox_rsi_eth.xml` - Ethernet configuration with external axis
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
`Config/User/Common/SensorInterface/rsi_3.3.x/gpios/` (KSS 8.3, 8.4)

**Files:**
- `b_ctrldbox_rsi_eth.xml` - Ethernet configuration with GPIO
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

| RSI version | KSS version | RSI selection | Source Folder | Files Deployed |
|---|---|---|---|---|
| **RSI 3.3.x** | 8.3, 8.4 | 1 (Standard) | `Config/User/Common/SensorInterface/rsi_3.3.x/` | `b_ctrldbox_rsi_eth.xml` + `b_ctrldbox_rsi.rsi` + `.rsi.diagram` + `.rsi.xml` |
| **RSI 3.3.x** | 8.3, 8.4 | 2 (External Axis) | `Config/User/Common/SensorInterface/rsi_3.3.x/ext_axis/` | External axis versions |
| **RSI 3.3.x** | 8.3, 8.4 | 3 (GPIO) | `Config/User/Common/SensorInterface/rsi_3.3.x/gpios/` | GPIO versions |
| **RSI 4.0.x** | 8.5 | 1 (Standard) | `Config/User/Common/SensorInterface/rsi_4.0.x/` | `b_ctrldbox_rsi_eth.xml` + `b_ctrldbox_rsi.rsix` |
| **RSI 4.0.x** | 8.5 | 2 (External Axis) | `Config/User/Common/SensorInterface/rsi_4.0.x/ext_axis/` | External axis versions |
| **RSI 4.0.x** | 8.5 | 3 (GPIO) | `Config/User/Common/SensorInterface/rsi_4.0.x/gpios/` | GPIO versions |
| **RSI 4.1.x** | 8.6 | 1 (Standard) | `Config/User/Common/SensorInterface/rsi_4.1.x/` | `b_ctrldbox_rsi_eth.xml` + `b_ctrldbox_rsi.rsix` |
| **RSI 4.1.x** | 8.6 | 2 (External Axis) | `Config/User/Common/SensorInterface/rsi_4.1.x/ext_axis/` | External axis versions |
| **RSI 4.1.x** | 8.6 | 3 (GPIO) | `Config/User/Common/SensorInterface/rsi_4.1.x/gpios/` | GPIO versions |

All configurations also deploy the same shared files regardless of RSI/KSS version:
- RSI program files (from `KRC/R1/Program/RSI/`)
- EKI config files (from `Config/User/Common/EthernetKRL/`)
- EKI server programs (from `KRC/R1/Program/EKIserver/`)

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

**For External Axis (RSI 4.0.x / KSS 8.5):**
```batch
copy Config\User\Common\SensorInterface\rsi_4.0.x\ext_axis\*.* C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
```

**For External Axis (RSI 4.1.x / KSS 8.6):**
```batch
copy Config\User\Common\SensorInterface\rsi_4.1.x\ext_axis\*.* C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
```

**For External Axis (RSI 3.3.x / KSS 8.3, 8.4):**
```batch
copy Config\User\Common\SensorInterface\rsi_3.3.x\ext_axis\*.* C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
```

**For GPIO (RSI 4.0.x / KSS 8.5):**
```batch
copy Config\User\Common\SensorInterface\rsi_4.0.x\gpios\*.* C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
```

**For GPIO (RSI 4.1.x / KSS 8.6):**
```batch
copy Config\User\Common\SensorInterface\rsi_4.1.x\gpios\*.* C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
```

**For GPIO (RSI 3.3.x / KSS 8.3, 8.4):**
```batch
copy Config\User\Common\SensorInterface\rsi_3.3.x\gpios\*.* C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
```

**Back to Standard (RSI 4.0.x / KSS 8.5):**
```batch
copy Config\User\Common\SensorInterface\rsi_4.0.x\b_ctrldbox_rsi_eth.xml C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
copy Config\User\Common\SensorInterface\rsi_4.0.x\b_ctrldbox_rsi.rsix C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
```

**Back to Standard (RSI 4.1.x / KSS 8.6):**
```batch
copy Config\User\Common\SensorInterface\rsi_4.1.x\b_ctrldbox_rsi_eth.xml C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
copy Config\User\Common\SensorInterface\rsi_4.1.x\b_ctrldbox_rsi.rsix C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
```

**Back to Standard (RSI 3.3.x / KSS 8.3, 8.4):**
```batch
copy Config\User\Common\SensorInterface\rsi_3.3.x\b_ctrldbox_rsi_eth.xml C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
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
