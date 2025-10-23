
ros2 launch b_controlled_box_launch ur_publish_description.launch.py \
  ur_type:=ur3e headless_mode:=true robot_ip:=192.168.56.101

set to OPERATING

ros2 control set_hardware_component_state -c /b_controlled_box_cm \
  ur3e active

load controllers:
ros2 launch b_controlled_box_launch load_controllers_for_the_scenario.launch.xml


sometimes, a controller is skipped. to launch a specific controller, do:
ros2 run controller_manager spawner -p scenario_controllers.yaml -c b_controlled_box_cm io_and_status_controller --inactive

activate some controllers:
ros2 control switch_controllers --activate joint_state_broadcaster joint_trajectory_controller -c b_controlled_box_cm

test joint_trajectory_controller
ros2 launch ur_robot_driver test_joint_trajectory_controller.launch.py


launch moveit:
must have description in one terminal
ros2 launch ur_robot_driver ur_rsp.launch.py ur_type:=ur3e robot_ip:=xx.xx.xx.xx

launch moveit in another
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e

