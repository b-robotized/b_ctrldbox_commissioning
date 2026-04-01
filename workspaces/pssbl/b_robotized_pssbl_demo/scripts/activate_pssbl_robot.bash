#!/bin/bash

# Load controllers
ros2 run controller_manager spawner -p scenario_controllers.yaml -c /b_controlled_box_cm joint_state_broadcaster --inactive
ros2 run controller_manager spawner -p scenario_controllers.yaml -c /b_controlled_box_cm tool_controller --inactive
ros2 run controller_manager spawner -p scenario_controllers.yaml -c /b_controlled_box_cm io_controller --inactive
ros2 run controller_manager spawner -p scenario_controllers.yaml -c /b_controlled_box_cm joint_trajectory_controller --inactive

# Connect to hardware - does also error reset - aftert this all motors should be blinking green
ros2 control set_hardware_component_state -c /b_controlled_box_cm orba6_ios active
ros2 control set_hardware_component_state -c /b_controlled_box_cm orba6_gripper active
ros2 control set_hardware_component_state -c /b_controlled_box_cm orba6 inactive

# Get state output from the hardware
ros2 control switch_controllers --activate joint_state_broadcaster -c /b_controlled_box_cm
ros2 control switch_controllers --activate io_controller -c /b_controlled_box_cm
ros2 control switch_controllers --activate tool_controller -c /b_controlled_box_cm

# Activate the hardware (enable movement)
ros2 control set_hardware_component_state -c /b_controlled_box_cm orba6 active

# Activate the trajectory controller that generates movement trajectories
ros2 control switch_controllers --activate joint_trajectory_controller -c /b_controlled_box_cm
