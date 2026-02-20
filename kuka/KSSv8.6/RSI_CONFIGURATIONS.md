# RSI Configuration Options

This document describes the different RSI configuration options available in the KSSv8.6 deployment.

## Overview

The b_ctrldbox RSI setup supports three different configurations:

1. **Standard** - Basic 6-axis robot control
2. **External Axis** - 6-axis robot + external axes (e.g., linear rails, positioners)
3. **GPIO** - 6-axis robot + GPIO digital I/O support

## Configuration Details

### 1. Standard Configuration

**Location:** `Config/User/Common/SensorInterface/` (root folder)

**Files:**
- `b_ctrldbox_rsi_eth.xml` - Ethernet configuration
- `b_ctrldbox_rsi.rsix` - RSI Visual diagram

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

**Location:** `Config/User/Common/SensorInterface/ext_axis/`

**Files:**
- `b_ctrldbox_rsi_eth.xml` - Ethernet configuration with external axis
- `b_ctrldbox_rsi.rsix` - RSI Visual diagram with AxisCorrExt

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

**Location:** `Config/User/Common/SensorInterface/gpios/`

**Files:**
- `b_ctrldbox_rsi_eth.xml` - Ethernet configuration with GPIO
- `b_ctrldbox_rsi.rsix` - RSI Visual diagram with GPIO blocks

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

When you run `deploy.bat`, you'll be prompted to select the configuration:

```
============================================
KUKA KSSv8.6 b_ctrldbox Deployment
============================================

Select RSI configuration:
  1. Standard (6 robot axes only)
  2. External Axis (6 robot axes + external axes support)
  3. GPIO (6 robot axes + GPIO support)

Enter selection (1/2/3):
```

### What Gets Deployed

Based on your selection:

| Selection | Source Folder | Files Deployed |
|-----------|---------------|----------------|
| **1 (Standard)** | `Config/User/Common/SensorInterface/` | Standard `b_ctrldbox_rsi_eth.xml` + `b_ctrldbox_rsi.rsix` |
| **2 (External Axis)** | `Config/User/Common/SensorInterface/ext_axis/` | External axis versions |
| **3 (GPIO)** | `Config/User/Common/SensorInterface/gpios/` | GPIO versions |

All configurations also deploy:
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

If you need to switch manually:

**For External Axis:**
```batch
copy Config\User\Common\SensorInterface\ext_axis\*.* C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
```

**For GPIO:**
```batch
copy Config\User\Common\SensorInterface\gpios\*.* C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
```

**Back to Standard:**
```batch
copy Config\User\Common\SensorInterface\b_ctrldbox_rsi_eth.xml C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
copy Config\User\Common\SensorInterface\b_ctrldbox_rsi.rsix C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
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
