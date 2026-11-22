#!/bin/bash

ros2 control set_hardware_component_state -c /b_controlled_box_cm left_FrankaHardwareInterface inactive
sleep 1
ros2 control set_hardware_component_state -c /b_controlled_box_cm right_FrankaHardwareInterface inactive
sleep 1
ros2 control set_hardware_component_state -c /b_controlled_box_cm left_FrankaHardwareInterface active
sleep 1
ros2 control set_hardware_component_state -c /b_controlled_box_cm right_FrankaHardwareInterface active
sleep 1
