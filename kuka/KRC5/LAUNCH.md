# Kuka KR5 ROS 2 Startup Procedure

This guide outlines the steps to launch the Kuka KR5 driver, spawn the necessary controllers, and activate the system for operation.

---

### Step 1: Launch the Robot Driver

First, launch the main driver for the Kuka robot. This command loads the robot's description (URDF) and starts the RSI (Robot Sensor Interface) hardware interface.

**Key Parameter:**

- `rsi_listen_ip`: This IP must match the address of the ctrlX CORE device that is on the same network subnet as the robot. The driver will open a port at this address to listen for the robot's state data.

**Command:**
```bash
ros2 launch kuka_rsi_driver load_description.launch.xml \
  description_package:=kuka_kr5_support \
  description_macro_file:=kr5_arc_macro.xacro \
  macro_name:=kuka_kr5_arc \
  rsi_listen_ip:=10.23.23.28 \
  rsi_listen_port:=28283
```

### Step 2: Activate the System

Before activating controllers, you must activate the hardware interface for RSI communication to be established.

#### Set ctrlX to SETUP Mode

⚠️ ***IMPORTANT:*** For real-time performance, switch the ctrlX controller to SETUP mode before proceeding.

#### Activate the Hardware Component

Transition the robot's hardware interface from an unconfigured to an active state.
```
ros2 control set_hardware_component_state -c /b_controlled_box_cm \
  kuka_kr5_arc active
```

### Step 3: Spawn Controllers

Next, spawn the actual controllers.

#### Spawn the Position Trajectory Controller

This controller is responsible for executing joint-space trajectories.

⚠️ ***IMPORTANT:*** Currently, for technical reasons, these commands must be executed in the same directory as this `LAUNCH.md` file.

```
ros2 run controller_manager spawner -c /b_controlled_box_cm \
  -p kuka_jtc.yaml position_trajectory_controller
```

#### Spawn the Joint State Broadcaster

This controller reads the current state of the robot's joints (position, velocity, etc.) from the hardware interface and publishes them to the /joint_states topic.

```
ros2 run controller_manager spawner -c /b_controlled_box_cm \
  -p joint_state_broadcaster.yaml joint_state_broadcaster
```
### Step 4.
Finally, run the test launch file to send a sample trajectory to the robot and verify that everything is working correctly.
```
ros2 launch kuka_ros2_control_support test_joint_trajectory_controller.launch.py

```

### Controller Switching
During operation, controllers can be switched to active or inactive state with the following commands:
```
ros2 control switch_controllers -c /b_controlled_box_cm \
  --activate position_trajectory_controller joint_state_broadcaster
```
```
ros2 control switch_controllers -c /b_controlled_box_cm \
  --deactivate position_trajectory_controller joint_state_broadcaster
```
It is possible to activate/deactivate more than one controllers in the same command

# Troubleshooting
To be added