#!/bin/bash

ros2 run controller_manager spawner -p scenario_controllers.yaml -c /b_controlled_box_cm joint_state_broadcaster
sleep 0.5
ros2 run controller_manager spawner -p scenario_controllers.yaml -c /b_controlled_box_cm joint_trajectory_controller
