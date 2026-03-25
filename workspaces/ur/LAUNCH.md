# UR ROS 2 Startup Procedure

This guide outlines the steps to launch the UR driver, spawn the necessary controllers, and activate the system for operation.

### Step 0: Observe controller manager activity

In one terminal output the `activity` topic of the controller manager to observe the internal states of the system:
```bash
ros2 topic echo /b_controlled_box_cm/activity
```

## Mock test
In one terminal, launch the controller manager:

```bash
ros2 launch b_robotized_ur_demo bringup_mock.launch.xml ur_type:=ur3e
```

In another, launch MoveIt and RViZ:
```bash
ros2 launch b_robotized_ur_demo moveit.launch.py ur_type:=ur3e
```
## Real robot

### Step 1: Launch the Robot Description

In a separate terminal, publish the description with the robot type you require.

**⚠️ Note:** *Currently, we must first get the path to configuration files on the CtrlX device. Yes it looks a bit weird. This will be obsolete soon, and we'll use a normal launch command for the UR description.*

```bash
ros2 topic echo --once --timeout 30 /b_controlled_box_cm/ctrlx/runtime_config_dir \
    | head -n 1 \
    | awk -F 'data: ' '{print $2}' \
    | xargs -I DIR ros2 launch b_robotized_ur_demo description.launch.xml \
        ur_type:=ur3e \
        runtime_config_dir:=DIR \
        robot_ip:=your-robot-ip
```

**Note:** *The "real robot" can we UrSim container as well. When running the UR simulator add argument robot_ip:=192.169.56.101*

Now you should see on the activity topic your component in the `unconfigured` state.

### Step 2: Set ctrlX to OPERATIONAL Mode

⚠️ ***IMPORTANT:*** For real-time performance, switch the ctrlX controller to OPERATIONAL mode before proceeding.

### Step 3: Activate the Robot

In third terminal activate the hardware interface. For this, make sure you're in the `scripts/` directory.

```bash
./activate_hardware.sh
```
   *As components are getting activated you will see new output on the `activity` topic.*

Then, spawn controllers

```bash
ros2 launch b_robotized_ur_demo spawn_controllers.launch.xml
```

Then, activate controllers:
```bash
./activate_controllers.sh
``` 

### Step 4: Run MoveIt

⚠️ ***IMPORTANT:*** Ensure you have robot description published in one terminal.

Start path planning framework MoveIt2 and visualization software `rviz2` using:

```bash
ros2 launch b_robotized_ur_demo moveit.launch.py ur_type:=ur3e
```

# Troubleshooting

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
joint_trajectory_controller
speed_scaling_state_broadcaster
force_torque_sensor_broadcaster
io_and_status_controller
scaled_joint_trajectory_controller
passthrough_trajectory_controller
force_mode_controller
freedrive_mode_controller
tool_contact_controller
tcp_pose_broadcaster
```

### Connection issues when trying to set the robot to `inactive` state.
Make sure that the IP addresses are set correctly and you can ping the robot.
To ping it choose `Setting` » `Network Diagnostics` » `Ping` on the ctrlX CORE and enter the address of the robot controller in the `Address` filed.

