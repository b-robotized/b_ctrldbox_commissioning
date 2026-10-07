# Kuka KRC5 ROS 2 Startup Procedure

This guide outlines the steps to launch the Kuka KRC5 driver, spawn the necessary controllers, and activate the system for operation.

---

### Step 1: Launch the Robot Driver

First, launch the main driver for the Kuka robot. This command loads the robot's description (URDF) and starts the RSI (Robot Sensor Interface) hardware interface.

**Key Parameters:**

- `robot_family/model:` This determines the exact robot model description and the corresponding macro. **Adjust to your desired model.** Supported models are in [`kuka_robot_descriptions` ros2 package](https://github.com/kroshu/kuka_robot_descriptions).
- `controller_ip` Is the name of the EKI interface IP for activating the robot.
- `client_ip`: This IP must match the address of the ctrlX CORE device that is on the same network subnet as the robot. The driver will open a port for RSI streaming at this address to listen for the robot's state data.

> **Note:** We currently support only `eki_rsi` driver version, which is most commonly used.

**Command:**
```bash
ros2 launch kuka_rsi_driver publish_description.launch.py \
robot_family:=agilus \
robot_model:=kr10_r900_2 \
controller_ip:=10.28.23.240 \
driver_version:=eki_rsi \
client_ip:=10.23.23.28 \
client_port:=28283 \
use_gpio:=false \
verify_robot_model:=false
```

### Step 2: Spawn the controllers

Before activating controllers, you must spawn the controllers which will manage the hardware interface.

⚠️ ***IMPORTANT:*** For technical reasons, for now, this must be launched from the same directory as this `LAUNCH.md` file.

Launch this in a separate terminal:

```bash
ros2 launch kuka_rsi_driver spawn_controllers.launch.py driver_version:=eki_rsi use_gpio:=false
```

#### Set ctrlX to OPERATIONAL Mode

⚠️ ***IMPORTANT:*** For real-time performance, switch the ctrlX controller to OPERATIONAL mode before proceeding.

### Step 3: Run the robot manager node

Kuka driver comes with a `robot_manager` node which activates controllers and hardware as part of the node lifecycle states.

Launch it in a separate terminal:

``` bash
ros2 launch kuka_rsi_driver robot_manager.launch.py robot_model:=kr10_r900_2 driver_version:=eki_rsi use_gpio:=true
```

### Step 4: Activate the robot

Lastly, in a separate terminal, configure and then activate the `robot_manager` node:

```bash
ros2 lifecycle set /robot_manager configure
```
```bash
ros2 lifecycle set /robot_manager activate
```


---

## Extended RSI configurations (torques, currents, status, Cartesian pose, GPIO)

The **Extended** and **Extended + GPIO** RSI configurations (options 4 and 5 of `deploy.bat`,
RSI 4.1.x / KSS 8.6 only - see [`kss_deployment/RSI_CONFIGURATIONS.md`](kss_deployment/RSI_CONFIGURATIONS.md))
send additional data that the driver only reads when it gets the matching RSI XML config YAML:

| RSI configuration | Driver YAML (`rsi_xml_config_file`) |
| :---------------- | :---------------------------------- |
| 1-3 (Standard / External Axis / GPIO) | not needed |
| 4 (Extended) | [`rsi_xml_config/extended.yaml`](rsi_xml_config/extended.yaml) |
| 5 (Extended + GPIO) | [`rsi_xml_config/extended_gpios.yaml`](rsi_xml_config/extended_gpios.yaml) |

The ethernet XML on the controller is generated from the same YAML, so always deploy the
controller files and pass the YAML of the **same** configuration.

> ⚠️ **ctrlX CORE:** the split launch files used above (`publish_description.launch.py`,
> `spawn_controllers.launch.py`) do not expose `rsi_xml_config_file` and the `read_*` arguments
> yet, so the Extended configurations can't be used through Steps 1-3 for now. Until the driver
> supports this, use the single-launch setup below (tested on a Linux PC).

### Single-launch setup (tested: KR 210 R3100-2, KSS 8.6.8, RSI 4.1.3, `rsi_only`)

```bash
ros2 launch kuka_rsi_driver startup.launch.py \
  driver_version:=rsi_only \
  robot_family:=quantec robot_model:=kr210_r3100_2 \
  client_ip:=10.23.23.28 client_port:=28283 \
  use_gpio:=false \
  rsi_xml_config_file:=<absolute path>/workspaces/kuka/rsi_xml_config/extended.yaml \
  read_robot_status:=true read_cartesian_pose:=true read_cartesian_setpoint:=true
```

- Adjust `robot_family`/`robot_model` to your robot.
- For configuration 5 use `extended_gpios.yaml` and `use_gpio:=true`; the robot description's
  GPIO interfaces (`gpio_config.xacro`) must match the GPIO names in that YAML.
- `read_current` no longer exists - motor current state interfaces are always exported.

Then activate as usual (with `rsi_only`, start the RSI program on the teach pendant within
10 s after `activate`):

```bash
ros2 lifecycle set robot_manager configure
ros2 lifecycle set robot_manager activate
```

The status and pose broadcasters are not spawned automatically - spawn them once the driver
is active:

```bash
ros2 run controller_manager spawner robot_status_broadcaster cartesian_pose_broadcaster cartesian_setpoint_broadcaster \
  -c controller_manager \
  --param-file $(ros2 pkg prefix --share kuka_rsi_driver)/config/kuka_cartesian_pose_broadcaster_config.yaml
```

### What to check

| Data | Where | Expected |
| :--- | :---- | :------- |
| Joint torques | `/joint_states` → `effort` | real values (not `NaN`), clearly non-zero on A2/A3 at standstill |
| Motor currents | `/dynamic_joint_states` → `current` | real values for every joint |
| Program state / speed scaling | `/robot_status_broadcaster/program_state`, `/robot_status_broadcaster/speed_scaling` | follows the program state and the override on the pendant |
| Cartesian pose | `/cartesian_pose_broadcaster/pose`, `/cartesian_setpoint_broadcaster/pose` | position in **metres**, TCP of the active tool in the active base |

The driver log must not contain the warnings "... is not configured in RSI XML" for the data
you enabled.
