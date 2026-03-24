#!/bin/bash
ros2 control switch_controllers -c /b_controlled_box_cm --activate joint_state_broadcaster
sleep 1
ros2 control switch_controllers -c /b_controlled_box_cm --activate joint_trajectory_controller