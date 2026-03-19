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

