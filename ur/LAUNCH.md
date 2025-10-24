Check in the first terminal the state of the controller manager within b»controlled box:

\\\
ros2 topic echo /b_controlled_box_cm/activity
\\\

In other terminal in container execute with the robot type you require.

\\\
ros2 launch b_controlled_box_launch ur_publish_description.launch.xml ur_type:=ur3e runtime_config_dir:ros2 topic echo --once --timeout 30 /b_controlled_box_cm/ctrlx/runtime_config_dir | head -n 1 | awk -F'data: ' '{print $2}'
\\\

For running the UR simulator add argument robot_ip:=192.169.56.101

Now you should see on the activity topic your component in the unconfigured state.

set to OPERATING

\\\
ros2 control set_hardware_component_state -c /b_controlled_box_cm ur3e active
\\\

Now you should see written with big friendly green leters RUNNING on your robot.

load controllers:

\\\
ros2 launch b_controlled_box_launch load_controllers_for_the_scenario.launch.xml
\\\

Now you should see bunch of controllers beeing in inactive state.

sometimes, a controller is skipped. to launch a specific controller, do:

\\\
ros2 run controller_manager spawner -p scenario_controllers.yaml -c b_controlled_box_cm io_and_status_controller --inactive
\\\

activate some controllers:

\\\
ros2 control switch_controllers --activate joint_state_broadcaster joint_trajectory_controller -c b_controlled_box_cm
\\\

test joint_trajectory_controller. before this, drive the robot to HOME position!

\\\
ros2 launch ur_robot_driver test_joint_trajectory\_controller.launch.py
\\\

### launch moveit:

must have description in one terminal

\\\
ros2 launch ur_robot_driver ur\_rsp.launch.py ur_type:=ur3e robot_ip:=xx.xx.xx.xx
\\\

## Add comment about ntp server. Install it into docker. I am using SNAP for now, but we can do it also manually. Simply test on the b»controlled box if the connection works. We should define docker always being 202 so we can make all default and nicely hardcoded so people don't have to fill out much. Maybe only ethernet interface name (can also try to determine it dynamically if there is only one). If not interface is defined in the .env file we should throw error when starting container, executing compose file.

launch moveit in another

\\\
ros2 launch b_controlled_box_launch ur\_moveit.launch.py ur_type:=ur3e
\\\
