# b_controlled_box_launch

Holds commands for launching ctrlx robot scenarios.
To be executed from `package://b_controlled_box_launch/launch` for ctrlx path reasons.

## Important!
WIP: config files must be in the same directory as the launch files, for ctrlx path reasons. Still figuring this out.

Build locally and execute:

### Start UR sim
```
ros2 run ur_client_library start_ursim.sh -m ur3e
```

### Publish UR description
```
ros2 launch b_controlled_box_launch ur_publish_description.launch.py ur_type:=ur3e headless_mode:=true robot_ip:=192.168.56.101
```

### Spawn `joint_state_broadcaster`
```
ros2 run controller_manager spawner -p joint_state_broadcaster.yaml -c /controller_manager joint_state_broadcaster
```

### Spawn `speed_scaling_state_broadcaster`
```
ros2 run controller_manager spawner -p ur_controllers.yaml -c /controller_manager speed_scaling_state_broadcaster
```

### Spawn `scaled_joint_trajectory_controller`
```
ros2 run controller_manager spawner -p ur_controllers.yaml -c /controller_manager joint_trajectory_controller
```

### Spawn `force_torque_sensor_broadcaster`
```
ros2 run controller_manager spawner -p ur_controllers.yaml -c /controller_manager force_torque_sensor_broadcaster
```

#### Other controllers available:
```
"joint_trajectory_controller",
"speed_scaling_state_broadcaster",
"force_torque_sensor_broadcaster",
"io_and_status_controller",
"scaled_joint_trajectory_controller",
"passthrough_trajectory_controller",
"force_mode_controller",
"freedrive_mode_controller",
"tool_contact_controller",
"tcp_pose_broadcaster"
```

### Test scaled_JTC` 
```
ros2 launch b_controlled_box_launch ur_controller_test.launch.py name:=joint_trajectory_controller
```