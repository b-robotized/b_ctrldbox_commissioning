# PSSBL ORBA6 ROS 2 Startup Procedure

This guide outlines the steps to launch the PSSBL ORBA6 robot (models 1215 and 2257).

**Available robot models:** `1215`, `2257` (default)

**Packages:**
- `pssbl_robot_descriptions` -- URDF, meshes, and robot description launch files
- `b_robotized_pssbl_demo` -- Bringup, MoveIt, controllers, and SRDF

### Step 0: Observe controller manager activity

In one terminal, output the `activity` topic of the controller manager to observe the internal states of the system:
```bash
ros2 topic echo /b_controlled_box_cm/activity
```

## View Robot Model Only

To just view the robot model in RViz with interactive joint sliders:
```bash
ros2 launch pssbl_robot_descriptions view_robot.launch.xml robot_model:=2257
```
or for the 1215 model:
```bash
ros2 launch pssbl_robot_descriptions view_robot.launch.xml robot_model:=1215
```

## Mock Test

In one terminal, launch the controller manager with mock hardware:
```bash
ros2 launch b_robotized_pssbl_demo bringup_mock.launch.xml robot_model:=2257
```

In another terminal, launch MoveIt and RViz:
```bash
ros2 launch b_robotized_pssbl_demo moveit.launch.xml robot_model:=2257
```

## Real Robot

### Step 1: Launch the Robot Description

In a separate terminal, publish the robot description with the desired hardware mode:
```bash
ros2 launch pssbl_robot_descriptions description.launch.xml robot_model:=2257 hardware:=motion_app
```

**Note:** Available hardware options are `mock`, `motion_app`, and `motor_direct`.

### Step 2: Set ctrlX to OPERATIONAL Mode

***IMPORTANT:*** For real-time performance, switch the ctrlX controller to OPERATIONAL mode before proceeding.

### Step 3: Activate the Robot

In a third terminal, activate the hardware interface. Make sure you are in the `scripts/` directory:
```bash
./activate_hardware.bash
```

Then, spawn controllers:
```bash
ros2 launch b_robotized_pssbl_demo spawn_controllers.launch.xml
```

Then, activate controllers:
```bash
./activate_controllers.bash
```

### Step 4: Run MoveIt

***IMPORTANT:*** Ensure you have the robot description published in one terminal.

Start the path planning framework MoveIt2 and visualization software `rviz2` using:
```bash
ros2 launch b_robotized_pssbl_demo moveit.launch.xml robot_model:=2257
```

## Troubleshooting

### Controller Switching

During operation, specific controllers can be switched to active or inactive state:
```bash
ros2 control switch_controllers -c /b_controlled_box_cm \
  --activate joint_state_broadcaster
```
```bash
ros2 control switch_controllers -c /b_controlled_box_cm \
  --deactivate joint_state_broadcaster
```

#### Available controllers:
```
joint_state_broadcaster
joint_trajectory_controller
tool_controller
```
