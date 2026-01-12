
# KASSOW ROS 2 Startup Procedure

This guide outlines the steps to launch the Kuka KRC4 driver, spawn the necessary controllers, and activate the system for operation.


# Single Kassow robot


### Step 0: Observe controller manager activity

In one terminal output the `activity` topic of the controller manager to observe the internal states of the system:
   ```
   ros2 topic echo /b_controlled_box_cm/activity
   ```

### Step 1: Launch the Robot Description

First, launch the description for the Kassow robot. This command loads the robot's description (URDF) to CtrlX and starts the Kassow hardware driver.

```bash
ros2 launch kassow_kord_bringup kassow_kord_description.launch.xml \
  use_mock_hardware:=false \
  ip_address:=10.23.23.238 \
  port:=28283
```

### Step 2: Set ctrlX to OPERATIONAL Mode

⚠️ ***IMPORTANT:*** For real-time performance, switch the ctrlX controller to OPERATIONAL mode before proceeding.

### Step 3: Activate the Robot

In third terminal load the controllers and active the whole system. For this enter the `scripts` folder of the `kassow_kord_bringup` package.
```bash
rosd kassow_kord_bringup && cd scripts
./activate_kassow_robot.bash
```
   *As components are getting activated you will see new output on the `activity` topic.*

### Step 4: Run MoveIt

Start path planning framework MoveIt2 and visualization software `rviz2` using:
```
ros2 launch kassow_kord_bringup kassow_kord_moveit.launch.xml
```




# Dual Kassow robot

The process for activation two robots is similar.

### Step 0: Observe controller manager activity

In one terminal output the `activity` topic of the controller manager to observe the internal states of the system:
   ```
   ros2 topic echo /b_controlled_box_cm/activity
   ```

### Step 1: Launch the Robot Description

First, launch the description for the Kassow robot. This command loads the robot's description (URDF) to CtrlX and starts the Kassow hardware driver.

Here, we differentiate between "`kassow_left`" and "`kassow_right`" robot.

```bash
ros2 launch kassow_kord_bringup kassow_kord_dual_arm_description.launch.xml \
  use_mock_hardware:=false \
  left_ip_address:=10.23.23.238 \
  left_port:=28283 \
  right_ip_address:=10.23.23.205 \
  right_port:=28284
```

### Step 2: Set ctrlX to OPERATIONAL Mode

⚠️ ***IMPORTANT:*** For real-time performance, switch the ctrlX controller to OPERATIONAL mode before proceeding.

### Step 3: Activate the Robot

In third terminal load the controllers and active the whole system. For this enter the `scripts` folder of the `kassow_kord_bringup` package.

```bash
rosd kassow_kord_bringup && cd scripts
```
*As components are getting activated you will see new output on the `activity` topic.*

#### Activate hardware interfaces
```bash
./dual_activate_hardware.bash
```
#### Activate controllers
```bash
./dual_activate_controllers.bash
```
   
### Step 4: Run MoveIt

Start path planning framework MoveIt2 and visualization software `rviz2` using:
```
ros2 launch kassow_kord_bringup kassow_kord_dual_arm_moveit.launch.xml
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

