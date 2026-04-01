#!/bin/bash

ros2 control set_hardware_component_state -c /b_controlled_box_cm orba6_ios active
ros2 control set_hardware_component_state -c /b_controlled_box_cm orba6_gripper active

ros2 control set_hardware_component_state -c /b_controlled_box_cm orba6 inactive

ros2 control set_hardware_component_state -c /b_controlled_box_cm orba6 active
