
# FANUC ROS 2 Startup Procedure

This guide outlines the steps to launch the Kuka KRC4 driver, spawn the necessary controllers, and activate the system for operation.


### Step 0: Observe controller manager activity

In one terminal output the `activity` topic of the controller manager to observe the internal states of the system:
   ```
   ros2 topic echo /b_controlled_box_cm/activity
   ```

## Mock test
In one terminal, launch the controller manager:

```bash
ros2 launch b_robotized_fanuc_demo fanuc_bringup_mock.launch.xml
```

In another, launch MoveIt and RViZ:
```bash
ros2 launch b_robotized_fanuc_demo fanuc_moveit.launch.xml
```

## Real robot

### Step 1: Launch the Robot Description

First, launch the description for the Fanuc robot. This command loads the robot's description (URDF) to CtrlX and starts the Fanuc hardware driver.

```bash
ros2 launch b_robotized_fanuc_demo fanuc_description.launch.xml \
  mock_hardware:=false \
  robot_ip:=192.168.0.100
```

### Step 2: Set ctrlX to OPERATIONAL Mode

⚠️ ***IMPORTANT:*** For real-time performance, switch the ctrlX controller to OPERATIONAL mode before proceeding.

### Step 3: Activate the Robot

In third terminal activate the hardware interface. For this, make sure you're in the `fanuc/` directory.

```bash
./activate_hardware.sh
```
   *As components are getting activated you will see new output on the `activity` topic.*

Then, activate controllers:
```bash
./activate_controllers.sh
``` 

### Step 4: Run MoveIt

Start path planning framework MoveIt2 and visualization software `rviz2` using:
```
ros2 launch b_robotized_fanuc_demo fanuc_moveit.launch.xml
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

### Connection issues when trying to set the robot to `inactive` state.
Make sure that the IP addresses are set correctly and you can ping the robot.
To ping it choose `Setting` » `Network Diagnostics` » `Ping` on the ctrlX CORE and enter the address of the robot controller in the `Address` filed.

