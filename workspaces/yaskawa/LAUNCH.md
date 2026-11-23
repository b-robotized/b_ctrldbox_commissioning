# Yaskawa NHC12 Startup Procedure

>⚠️ ***IMPORTANT:*** This package supports only the NHC12 robot for now.

This guide shows how to launch the NHC12 with `yaskawa_acu_bringup`, activate the controllers and run MoveIt.

[Foreman](https://github.com/ros-controls/foreman) switches the hardware and the controllers to the states of a profile. The bringup launch files launch it. Read the foreman documentation to see how it works and how to use it.

## Mock hardware

The mock launch file runs its own controller manager. Foreman activates the controllers automatically.

1. Launch the bringup:

   ```bash
   ros2 launch yaskawa_acu_bringup yaskawa_acu_bringup_mock.launch.xml
   ```

2. In a second terminal, launch MoveIt and RViz:

   ```bash
   ros2 launch yaskawa_acu_bringup yaskawa_acu_moveit.launch.xml
   ```

## Real hardware

On the real robot, the controller manager runs on the RT PC with the name `b_controlled_box_cm`. The bringup launch file does not run it.

### Step 0: Run the zenoh router (optional)

If you use zenoh, run the router with the IP address of the remote router:

```bash
rtw-zenoh-router <IP-ADDRESS>
```

Leave it running in that terminal.

### Step 1: Observe the controller manager

In one terminal, show the `activity` topic to observe the state of the system:

```bash
ros2 topic echo /b_controlled_box_cm/activity
```

### Step 2: Run the controller manager

Run the controller manager on the RT PC. Wait until it runs.

### Step 3: Launch the bringup

This command publishes the robot description. It launches the controller spawners and foreman. The spawners load the controllers in the `inactive` state.

```bash
ros2 launch yaskawa_acu_bringup yaskawa_acu_bringup.launch.xml
```

The robot IP address and port are the defaults in `yaskawa_acu_bringup/launch/yaskawa_acu_description.launch.xml` (`192.168.1.253`, `50300`). Change them there.

### Step 4: Set ctrlX to OPERATIONAL mode

> ⚠️ ***IMPORTANT:*** For real-time performance, switch the ctrlX controller to OPERATIONAL mode before you continue.

### Step 5: Activate the robot

In a third terminal, set foreman to the `active` profile. This activates the hardware interface and the controllers:

```bash
ros2 action send_goal --feedback /foreman/set_profile foreman_msgs/action/SetProfile "{profile: 'active'}"
```

New output shows on the `activity` topic while the components activate.

### Step 6: Run MoveIt

Launch MoveIt and RViz:

```bash
ros2 launch yaskawa_acu_bringup yaskawa_acu_moveit.launch.xml
```

In RViz, select the `ompl` or `pilz` planning pipeline. Plan a motion. Then execute it.

## Troubleshooting

### Controller switching

Sometimes a controller does not activate. Use these commands to switch the controllers:

```bash
ros2 control switch_controllers -c /b_controlled_box_cm --activate joint_state_broadcaster
ros2 control switch_controllers -c /b_controlled_box_cm --deactivate joint_state_broadcaster
```

You can switch more than one controller in the same command.

Available controllers:

```
joint_state_broadcaster
joint_trajectory_controller
```

### Connection problems

Make sure that the IP address is correct and that you can ping the robot.
To ping it, choose `Settings` » `Network Diagnostics` » `Ping` on the ctrlX CORE. Enter the address of the robot controller in the `Address` field.
