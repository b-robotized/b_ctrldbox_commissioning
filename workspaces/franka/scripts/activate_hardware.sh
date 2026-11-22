#!/bin/bash

ros2 control set_hardware_component_state -c /b_controlled_box_cm FrankaHardwareInterface inactive
sleep 1
ros2 control set_hardware_component_state -c /b_controlled_box_cm FrankaHardwareInterface active
sleep 1
