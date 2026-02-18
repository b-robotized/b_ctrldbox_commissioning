#!/bin/bash

ros2 run controller_manager spawner -p scenario_controllers.yaml -c /b_controlled_box_cm joint_state_broadcaster

ros2 run controller_manager spawner -p scenario_controllers.yaml -c /b_controlled_box_cm fanuc_joint_trajectory_controller --inactive
ros2 run controller_manager spawner -p scenario_controllers.yaml -c /b_controlled_box_cm fanuc_gpio_controller --inactive

ros2 control switch_controllers -c /b_controlled_box_cm --activate fanuc_joint_trajectory_controller
ros2 control switch_controllers -c /b_controlled_box_cm --activate fanuc_gpio_controller