```
# listen_ip tells the Kuka ros2 driver where to open a port for listening to robot.
ros2 launch kuka_rsi_driver load_description.launch.xml description_package:=kuka_kr5_support description_macro_file:=kr5_arc_macro.xacro macro_name:=kuka_kr5_arc rsi_listen_ip:=192.168.28.202
```

```
ros2 run controller_manager spawner -p kuka_jtc.yaml -c /controller_manager position_trajectory_controller
```

```
ros2 run controller_manager spawner -p joint_state_broadcaster.yaml -c /controller_manager joint_state_broadcaster
```

**Before activating:** switch to `SETUP` on ctrlx for real-timedness

```
ros2 control set_hardware_component_state kuka_kr5_arc active
```

```
ros2 control switch_controllers activate position_trajectory_controller joint_state_broadcaster
```

```
ros2 launch kuka_ros2_control_support test_joint_trajectory_controller.launch.py
```