#!/bin/bash

ros2 control switch_controllers --deactivate joint_trajectory_controller -c /b_controlled_box_cm
sleep 0.5
ros2 control switch_controllers --deactivate joint_state_broadcaster -c /b_controlled_box_cm
sleep 0.5

ros2 control set_hardware_component_state -c /b_controlled_box_cm orba6_ios inactive
sleep 0.5
ros2 control set_hardware_component_state -c /b_controlled_box_cm orba6 inactive
sleep 0.5

ros2 control set_hardware_component_state -c /b_controlled_box_cm orba6_ios unconfigured
sleep 0.5
ros2 control set_hardware_component_state -c /b_controlled_box_cm orba6 unconfigured
