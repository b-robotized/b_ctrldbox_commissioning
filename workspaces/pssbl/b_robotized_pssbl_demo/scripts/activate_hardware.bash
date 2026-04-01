#!/bin/bash

ros2 control set_hardware_component_state -c /b_controlled_box_cm orba6_ios active
sleep 0.5

ros2 control set_hardware_component_state -c /b_controlled_box_cm orba6 inactive
sleep 0.5
ros2 control set_hardware_component_state -c /b_controlled_box_cm orba6 active
