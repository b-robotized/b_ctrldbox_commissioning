#!/bin/bash

# Load controllers
ros2 run controller_manager spawner -p scenario_controllers.yaml -c /b_controlled_box_cm joint_state_broadcaster --inactive
sleep 0.5
ros2 run controller_manager spawner -p scenario_controllers.yaml -c /b_controlled_box_cm joint_trajectory_controller --inactive
sleep 0.5

# Connect to hardware - does also error reset - aftert this all motors should be blinking green
ros2 control set_hardware_component_state -c /b_controlled_box_cm orba6_ios active
sleep 0.5
ros2 control set_hardware_component_state -c /b_controlled_box_cm orba6 inactive
sleep 0.5

# Get state output from the hardware
ros2 control switch_controllers --activate joint_state_broadcaster -c /b_controlled_box_cm
sleep 0.5

# Activate the hardware (enable movement)
ros2 control set_hardware_component_state -c /b_controlled_box_cm orba6 active
sleep 0.5

# Activate the trajectory controller that generates movement trajectories
ros2 control switch_controllers --activate joint_trajectory_controller -c /b_controlled_box_cm
