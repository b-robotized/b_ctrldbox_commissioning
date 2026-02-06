
# Dobot Nova2 ROS 2 Startup Procedure

### Step 0: Observe controller manager activity

In one terminal output the `activity` topic of the controller manager to observe the internal states of the system:
   ```
   ros2 topic echo /controller_manager/activity
   ```

### Step 1: Launch the Robot Description

First, launch the description for the Kassow robot. This command loads the robot's description (URDF) to CtrlX and starts the Kassow hardware driver.

```bash
ros2 launch cr_robot_ros2 dobot_bringup.launch.xml use_mock_hardware:=true robot_type:=nova2
```