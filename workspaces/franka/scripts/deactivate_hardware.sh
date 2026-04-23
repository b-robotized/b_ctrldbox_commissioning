#!/bin/bash

ros2 control set_hardware_component_state -c /b_controlled_box_cm fr3_FrankaHardwareInterface inactive
sleep 1
