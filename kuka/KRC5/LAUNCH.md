CD to this directory

listen_ip tells the Kuka ros2 driver where to open a port for listening to robot. 
This should be the IP and port on the CtrlX CORE device which is connected to the robot, on the same subnet as the robot.

```
ros2 launch kuka_rsi_driver load_description.launch.xml description_package:=kuka_kr5_support description_macro_file:=kr5_arc_macro.xacro macro_name:=kuka_kr5_arc rsi_listen_ip:=10.23.23.28 rsi_listen_port:=28283 
```

```
ros2 run controller_manager spawner -c /b_controlled_box_cm -p kuka_jtc.yaml  position_trajectory_controller
```

```
ros2 run controller_manager spawner -c /b_controlled_box_cm -p joint_state_broadcaster.yaml joint_state_broadcaster
```

**Before activating:** switch to `SETUP` on ctrlx for real-timedness

```
ros2 control set_hardware_component_state -c /b_controlled_box_cm kuka_kr5_arc active 
```

```
ros2 control switch_controllers -c /b_controlled_box_cm --activate position_trajectory_controller joint_state_broadcaster 
```

```
ros2 launch kuka_ros2_control_support test_joint_trajectory_controller.launch.py
```