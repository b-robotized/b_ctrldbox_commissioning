
# Dobot Nova2 ROS 2 Startup Procedure

### Step 0: Observe controller manager activity

In one terminal output the `activity` topic of the controller manager to observe the internal states of the system:
   ```
   ros2 topic echo /controller_manager/activity
   ```

### Step 1: Launch the MOCK Robot Bringup

First, launch the description for the Kassow robot. This command loads the robot's description (URDF) to CtrlX and starts the Kassow hardware driver.

```bash
ros2 launch cr_robot_ros2 dobot_bringup.launch.xml use_mock_hardware:=true robot_type:=nova2
```

### Step 2: Test gripper

Send the gripper action command. For this gripper, closed/open positions are `[-0.037, 0.01]`

```bash
ros2 action send_goal /feetech_gripper_controller/gripper_cmd control_msgs/action/ParallelGripperCommand "{command: {position: [-0.037]}}" 
```

### Step 3: Launch real robot:
Ensure parameter `use_mock_hardware:=false`

```bash
ros2 launch cr_robot_ros2 dobot_bringup.launch.xml use_mock_hardware:=false robot_type:=nova2
```