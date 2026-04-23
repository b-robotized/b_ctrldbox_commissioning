# Franka ROS 2 Startup Procedure

This guide outlines the steps to launch the Franka driver, spawn the necessary controllers, and activate the system for operation.

## Start `zenoh` router
if using zenoh, make sure to start the zenoh router with IP address of the remote router on another device you're targeting:

```bash
rtw-zenoh-router <IP-ADDRESS>
```
And leave it running in that terminal.

## Single Franka Robot

### Step 0: Observe controller manager activity

In one terminal output the `activity` topic of the controller manager to observe the internal states of the system:
```
ros2 topic echo /b_controlled_box_cm/activity
```

### Step 1: Launch the Robot Description

First, launch the description for the Franka robot. This command loads the robot's description (URDF) to CtrlX and starts the Franka hardware driver.

```bash
ros2 launch b_robotized_franka_demo franka_description.launch.xml \
  use_mock_hardware:=false \
  robot_ip:=10.42.0.203 \
  arm_id:=fr3
```

### Step 2: Set ctrlX to OPERATIONAL Mode

⚠️ ***IMPORTANT:*** For real-time performance, switch the ctrlX controller to OPERATIONAL mode before proceeding.

### Step 3: Activate the Robot

In third terminal activate the hardware interface and controllers. For this, make sure you're in the `scripts/` directory.

```bash
cd scripts
./activate_franka.sh
```
*As components are getting activated you will see new output on the `activity` topic.*

### Step 4: Run MoveIt

⚠️ ***IMPORTANT:*** Ensure you have robot description published in one terminal.

Start path planning framework MoveIt2 and visualization software `rviz2` using:

```bash
ros2 launch b_robotized_franka_demo franka_moveit.launch.xml
```

## Dual Franka Robot

The process for activating two robots is similar.

### Step 0: Observe controller manager activity

In one terminal output the `activity` topic of the controller manager to observe the internal states of the system:
```
ros2 topic echo /b_controlled_box_cm/activity
```

### Step 1: Launch the Robot Description

First, launch the description for the Franka robots. This command loads the robot's description (URDF) to CtrlX and starts the Franka hardware drivers.

Here, we differentiate between "`fr3_left`" and "`fr3_right`" robot.

```bash
ros2 launch b_robotized_franka_demo franka_dual_arm_description.launch.xml \
  use_mock_hardware:=false \
  robot_1_ip:=10.42.0.203 \
  robot_2_ip:=10.42.0.204
```

### Step 2: Set ctrlX to OPERATIONAL Mode

⚠️ ***IMPORTANT:*** For real-time performance, switch the ctrlX controller to OPERATIONAL mode before proceeding.

### Step 3: Activate the Robot

In third terminal activate the hardware interface and controllers. For this, make sure you're in the `scripts/` directory.

```bash
cd scripts
./activate_franka_dual.sh
```
*As components are getting activated you will see new output on the `activity` topic.*

### Step 4: Run MoveIt

⚠️ ***IMPORTANT:*** Ensure you have robot description published in one terminal.

Start path planning framework MoveIt2 and visualization software `rviz2` using:

```bash
ros2 launch b_robotized_franka_demo franka_dual_arm_moveit.launch.xml
```

## Mock Test

### Single Robot
In one terminal, launch the controller manager:

```bash
ros2 launch b_robotized_franka_demo franka_bringup_mock.launch.xml
```

In another, launch MoveIt and RViZ:
```bash
ros2 launch b_robotized_franka_demo franka_moveit.launch.xml
```

### Dual Robot
In one terminal, launch the controller manager:

```bash
ros2 launch b_robotized_franka_demo franka_dual_arm_bringup_mock.launch.xml
```

In another, launch MoveIt and RViZ:
```bash
ros2 launch b_robotized_franka_demo franka_dual_arm_moveit.launch.xml
```

# Troubleshooting

### Recovering from an Error
- limit break, communication timeout or other.
This breaks the communication of the robot and we have to deactivate and reconfigure it.

0. Clear the errors on the robot teach pendant and re-activate.

1. deactivate the robot hardware and controllers.
```
cd scripts
./deactivate_hardware.sh
```
2. unconfigure the controllers
```
ros2 control set_controller_state -c b_controlled_box_cm joint_state_broadcaster unconfigured
ros2 control set_controller_state -c b_controlled_box_cm fr3_joint_trajectory_controller unconfigured
```
3. unconfigure the hardware
```
ros2 control set_hardware_component_state -c /b_controlled_box_cm fr3_FrankaHardwareInterface unconfigured
```
4. reactivate the robot
```
./activate_franka.sh
```

### Controller Switching
During operation, some controller activation service might fail. In that case, specific controllers can be switched to active or inactive state with the following commands:
```
ros2 control switch_controllers -c /b_controlled_box_cm \
  --activate joint_state_broadcaster
```
```
ros2 control switch_controllers -c /b_controlled_box_cm \
  --deactivate joint_state_broadcaster
```
It is possible to activate/deactivate more than one controllers in the same command

#### Available controllers:
```
fr3_joint_trajectory_controller (single robot)
fr3_left_joint_trajectory_controller (dual robot - left)
fr3_right_joint_trajectory_controller (dual robot - right)
```

### Connection issues when trying to set the robot to `inactive` state.
Make sure that the IP addresses are set correctly and you can ping the robot.
To ping it choose `Setting` » `Network Diagnostics` » `Ping` on the ctrlX CORE and enter the address of the robot controller in the `Address` field.