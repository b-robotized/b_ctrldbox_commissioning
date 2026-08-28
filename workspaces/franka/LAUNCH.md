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

### Step 1: Launch the Robot Bringup

First, launch the description for the Franka robot. This command loads the robot's description (URDF) to CtrlX and starts the Franka hardware driver.

```bash
ros2 launch b_robotized_franka_demo franka_bringup.launch.xml robot_ip:=10.42.0.203
```

### Step 2: Set ctrlX to OPERATIONAL Mode

⚠️ ***IMPORTANT:*** For real-time performance, switch the ctrlX controller to OPERATIONAL mode before proceeding.

### Step 3: Activate the Robot

In third terminal activate the hardware interface and controllers. For this, transition foreman to `active` state:

```bash
ros2 action send_goal --feedback /foreman/set_profile foreman_msgs/action/SetProfile "{profile: 'active'}"
```
*As components are getting activated you will see new output on the `activity` topic.*

### Step 4: Run MoveIt

Start path planning framework MoveIt2 and visualization software `rviz2` using:

```bash
ros2 launch b_robotized_franka_demo franka_moveit.launch.xml
```

## Dual Franka Robot

The process for activating two robots is similar. Currently, only `mock` hardware is possible for dual franka in this example workspace

### Step 0: Observe controller manager activity

In one terminal output the `activity` topic of the controller manager to observe the internal states of the system:
```
ros2 topic echo /b_controlled_box_cm/activity
```

### Step 1: Launch the Robot Description

First, launch the description for the Franka robots. This command loads the robot's description (URDF) to CtrlX and starts the Franka hardware drivers.

Here, we differentiate between "`fr3_left`" and "`fr3_right`" robot.

```bash
ros2 launch b_robotized_franka_demo franka_dual_arm_bringup_mock.launch.xml
```

### Step 2: Activate the Robot

In third terminal activate the hardware interface and controllers. For this, transition foreman to `active` state:

```bash
ros2 action send_goal --feedback /foreman/set_profile foreman_msgs/action/SetProfile "{profile: 'active'}"
```

*As components are getting activated you will see new output on the `activity` topic.*

### Step 4: Run MoveIt

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
In third terminal activate the hardware interface and controllers. For this, transition foreman to `active` state:

```bash
ros2 action send_goal --feedback /foreman/set_profile foreman_msgs/action/SetProfile "{profile: 'active'}"
```

# Troubleshooting

### Recovering from an Error
Refer to troubleshooting entry in [this portion of the docs.](../../docs/supported_robots/FRANKA.md)

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
joint_state_broadcaster
fr3_joint_trajectory_controller (single robot)
fr3_left_joint_trajectory_controller (dual robot - left)
fr3_right_joint_trajectory_controller (dual robot - right)
```

### Connection issues when trying to set the robot to `inactive` state.
Make sure that the IP addresses are set correctly and you can ping the robot.
To ping it choose `Setting` » `Network Diagnostics` » `Ping` on the ctrlX CORE and enter the address of the robot controller in the `Address` field.