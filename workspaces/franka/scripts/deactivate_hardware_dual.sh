#!/bin/bash

ros2 control set_hardware_component_state -c /b_controlled_box_cm fr3_left_FrankaHardwareInterface inactive
sleep 1
ros2 control set_hardware_component_state -c /b_controlled_box_cm fr3_right_FrankaHardwareInterface inactive
sleep 1
