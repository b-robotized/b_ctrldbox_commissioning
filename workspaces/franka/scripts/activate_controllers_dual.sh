#!/bin/bash

ros2 run controller_manager spawner -p scenario_controllers.yaml -c /b_controlled_box_cm joint_state_broadcaster
sleep 1

ros2 run controller_manager spawner -p scenario_controllers.yaml -c /b_controlled_box_cm fr3_left_joint_trajectory_controller --inactive
sleep 1
ros2 run controller_manager spawner -p scenario_controllers.yaml -c /b_controlled_box_cm fr3_right_joint_trajectory_controller --inactive
sleep 1

ros2 control switch_controllers -c /b_controlled_box_cm --activate fr3_left_joint_trajectory_controller
sleep 1
ros2 control switch_controllers -c /b_controlled_box_cm --activate fr3_right_joint_trajectory_controller
