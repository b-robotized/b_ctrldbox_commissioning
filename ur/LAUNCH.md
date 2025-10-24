Check in the first terminal the state of the controller manager within b»controlled box:

```
ros2 topic echo /b_controlled_box_cm/activity
```

In other terminal in container execute with the robot type you require.

**Note:** *Currently, we must first get the path to configuration files on the CtrlX device. This will be obsolete soon, and we'll use a normal launch command for the UR description.*
```
ros2 topic echo --once --timeout 30 /b_controlled_box_cm/ctrlx/runtime_config_dir \
    | head -n 1 \
    | awk -F 'data: ' '{print $2}' \
    | xargs -I DIR ros2 launch b_controlled_box_launch ur_publish_description.launch.xml \
        ur_type:=ur3e \
        runtime_config_dir:=DIR
```

**Note:** *For running the UR simulator add argument robot_ip:=192.169.56.101*

Now you should see on the activity topic your component in the unconfigured state.

set to OPERATING

```
ros2 control set_hardware_component_state -c /b_controlled_box_cm ur3e active
```

Now you should see written with big friendly green leters RUNNING on your robot.

load controllers:

```
ros2 launch b_controlled_box_launch load_controllers_for_the_scenario.launch.xml
```

Now you should see bunch of controllers beeing in inactive state.

sometimes, a controller is skipped. to launch a specific controller, do:

```
ros2 run controller_manager spawner -p scenario_controllers.yaml -c b_controlled_box_cm io_and_status_controller --inactive
```

activate some controllers:

```
ros2 control switch_controllers --activate joint_state_broadcaster joint_trajectory_controller -c b_controlled_box_cm
```

test joint_trajectory_controller. before this, drive the robot to HOME position!

```
ros2 launch ur_robot_driver test_joint_trajectory_controller.launch.py
```

### launch moveit:

must have description in one terminal

```
ros2 launch ur_robot_driver ur_rsp.launch.py ur_type:=ur3e robot_ip:=xx.xx.xx.xx
```
launch moveit in another

```
ros2 launch b_controlled_box_launch ur\_moveit.launch.py ur_type:=ur3e
```
