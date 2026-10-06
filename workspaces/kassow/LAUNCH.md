# KASSOW ROS 2 Startup Procedure

This guide outlines the steps to launch the hardware drivers, spawn the necessary controllers, and activate the system for operation using the Foreman profile manager.

## Supported Robot Models
The KORD bringup currently supports two Kassow robot models:
* **`kr810`** (Default)
* **`kr1018`**

You can specify the model during the launch process by passing the corresponding argument. If no argument is provided, the system defaults to the `kr810`.

## Single Kassow Robot

### Step 0: Observe Controller Manager Activity
In one terminal, output the `activity` topic of the controller manager to observe the internal states of the system:
```bash
ros2 topic echo /b_controlled_box_cm/activity
```

### Step 1: Launch the Robot Bringup
Launch the bringup file for the Kassow robot. This command loads the robot's description (URDF), starts the hardware driver, and initializes Foreman. Pass the `robot_model` argument to specify the exact robot model.

**For Real Hardware:**
```bash
ros2 launch kassow_kord_bringup kassow_kord_bringup.launch.xml \
  robot_model:=kr1018
```

**For Mock Hardware:**
```bash
ros2 launch kassow_kord_bringup kassow_kord_bringup_mock.launch.xml \
  robot_model:=kr1018
```

### Step 2: Set ctrlX to OPERATIONAL Mode
⚠️️ **IMPORTANT:** For real-time performance on physical hardware, switch the ctrlX controller to OPERATIONAL mode before proceeding.

### Step 3: Activate the Robot via Foreman
In a new terminal, use Foreman to load the controllers and activate the system by setting the profile to `active`:
```bash
ros2 service call /foreman/set_profile foreman_msgs/srv/SetProfile "{profile: 'active'}"
```
*As components are activated, you will see new output on the `activity` topic.*

### Step 4: Run MoveIt
Start the planning framework MoveIt2 and visualization software `rviz2`. Pass the same `robot_model` argument to ensure correct SRDF is loaded:
```bash
ros2 launch kassow_kord_bringup kassow_kord_moveit.launch.xml \
  robot_model:=kr1018
```

---

## Dual Kassow Robot

The process for activating two robots follows the same Foreman-based profile structure. You can mix and match robot models by specifying `robot_1_model` (left arm) and `robot_2_model` (right arm) independently.

### Step 0: Observe Controller Manager Activity
In one terminal, output the `activity` topic of the controller manager to observe the internal states of the system:
```bash
ros2 topic echo /b_controlled_box_cm/activity
```

### Step 1: Launch the Dual Robot Bringup
Launch the dual-arm bringup configuration. This loads the descriptions and hardware interfaces for both the `kassow_left` and `kassow_right` robots.

**For Real Hardware:**
```bash
ros2 launch kassow_kord_bringup kassow_kord_dual_arm_bringup.launch.xml \
  robot_1_model:=kr810 \
  robot_2_model:=kr1018
```

**For Mock Hardware:**
```bash
ros2 launch kassow_kord_bringup kassow_kord_dual_arm_bringup_mock.launch.xml \
  robot_1_model:=kr810 \
  robot_2_model:=kr1018
```

### Step 2: Set ctrlX to OPERATIONAL Mode
⚠️ **IMPORTANT:** For real-time performance on physical hardware, switch the ctrlX controller to OPERATIONAL mode before proceeding.

### Step 3: Activate the Robots via Foreman
In a new terminal, activate the hardware interfaces and controllers for both robots simultaneously using Foreman:
```bash
ros2 service call /foreman/set_profile foreman_msgs/srv/SetProfile "{profile: 'active'}"
```

### Step 4: Run MoveIt
Start the dual-arm path planning framework MoveIt2 and visualization software `rviz2`. Make sure to pass the matching `robot_1_model` and `robot_2_model` parameters:
```bash
ros2 launch kassow_kord_bringup kassow_kord_dual_arm_moveit.launch.xml \
  robot_1_model:=kr810 \
  robot_2_model:=kr1018
```

---

## Troubleshooting

### Recovering from a CBun Error
A limit break, communication timeout, or other issue will break the communication of the robot. Foreman handles the state transitions to cleanly deactivate and reconfigure the controllers.

1. Clear the errors on the robot teach pendant and re-activate CBun.
2. Deactivate the robot hardware and controllers using Foreman:
   ```bash
   ros2 service call /foreman/set_profile foreman_msgs/srv/SetProfile "{profile: 'inactive'}"
   ```
3. Reactivate the robot:
   ```bash
   ros2 service call /foreman/set_profile foreman_msgs/srv/SetProfile "{profile: 'active'}"
   ```

### Controller Switching
If specific controllers fail or require manual intervention outside of Foreman's automated profiles, they can still be switched directly via the controller manager:
```bash
ros2 control switch_controllers -c /b_controlled_box_cm --activate joint_state_broadcaster
```
```bash
ros2 control switch_controllers -c /b_controlled_box_cm --deactivate joint_state_broadcaster
```

### Connection Issues
If there are connection issues when trying to set the robot to the `inactive` state, ensure the IP addresses are correct and the robot is pingable. To ping it, navigate to **Settings » Network Diagnostics » Ping** on the ctrlX CORE and enter the address of the robot controller in the `Address` field.