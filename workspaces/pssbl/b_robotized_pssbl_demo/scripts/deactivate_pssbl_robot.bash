#!/bin/bash

ros2 control switch_controllers --deactivate joint_trajectory_controller -c /b_controlled_box_cm
ros2 control switch_controllers --deactivate joint_state_broadcaster -c /b_controlled_box_cm
ros2 control switch_controllers --deactivate io_controller -c /b_controlled_box_cm
ros2 control switch_controllers --deactivate tool_controller -c /b_controlled_box_cm

ros2 control set_hardware_component_state -c /b_controlled_box_cm orba6_ios inactive
ros2 control set_hardware_component_state -c /b_controlled_box_cm orba6_gripper inactive
ros2 control set_hardware_component_state -c /b_controlled_box_cm orba6 inactive

ros2 control set_hardware_component_state -c /b_controlled_box_cm orba6_ios unconfigured
ros2 control set_hardware_component_state -c /b_controlled_box_cm orba6_gripper unconfigured
ros2 control set_hardware_component_state -c /b_controlled_box_cm orba6 unconfigured
