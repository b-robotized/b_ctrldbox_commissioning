# Kuka KRC5 ROS 2 Startup Procedure

This guide outlines the steps to launch the Kuka KRC4 driver, spawn the necessary controllers, and activate the system for operation.

---

### Step 1: Launch the Robot Driver

First, launch the main driver for the Kuka robot. This command loads the robot's description (URDF) and starts the RSI (Robot Sensor Interface) hardware interface.

**Key Parameters:**

- `rsi_listen_ip`: This IP must match the address of the ctrlX CORE device that is on the same network subnet as the robot. The driver will open a port at this address to listen for the robot's state data.
- `description_package:` This determines the exact robot model description and the corresponding macro. **Adjust to your desired model.** Supported models are in [`kuka_experimental` ros2 package](https://github.com/b-robotized-forks/kuka_experimental).
  - `macro_name` will be the name of the hardware interface for the activation command below.

**Command:**
```bash
ros2 launch kuka_rsi_driver load_description.launch.xml \
  description_package:=kuka_kr3_support \
  description_macro_file:=kr3r540_macro.xacro \
  macro_name:=kuka_kr3r540 \
  rsi_listen_ip:=10.23.23.28 \
  rsi_listen_port:=28283
```

### Step 2: Activate the System

Before activating controllers, you must activate the hardware interface for RSI communication to be established.

#### Set ctrlX to SETUP Mode

⚠️ ***IMPORTANT:*** For real-time performance, switch the ctrlX controller to SETUP mode before proceeding.

#### Activate the Hardware Component

Transition the robot's hardware interface from an unconfigured to an active state.

⚠️️ ***IMPORTANT:*** Ensure your hardware name fits the `macro_name` set in the description. Here it is `kuka_kr3r540`.

```
ros2 control set_hardware_component_state -c /b_controlled_box_cm \
  kuka_kr3r540 active
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
⚠️️ ***IMPORTANT:*** Depending on the robot's initial position, due to the test node implementation, this test might fail. This will be corrected in future versions of `publisher_joint_trajectory_controller` node. For now, the expected initial position is:
```
[0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
```
... and can be checked by runnin `ros2 topic echo /joint_states` after spawning `joint_state_broadcaster`. If issues arise, contact b-robotized support.

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