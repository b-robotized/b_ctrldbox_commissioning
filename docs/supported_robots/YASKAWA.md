# Yaskawa Robot Setup

> ⚠️ ***NOTE:*** This page is not complete. We will edit it further.

Only the Yaskawa MOTOMAN NHC12 with the NEXT ACU controller is supported now.

The rest of the manual assumes that you set up your ctrlX CORE to the [standard setup for the b»controlled box App](../SETUP_CTRLX.md).

## Connection

The b»controlled box connects to the ACU controller with gRPC.

- Default address: `192.168.1.253`
- Default port: `50300`

To change them, edit `yaskawa_acu_bringup/launch/yaskawa_acu_description.launch.xml`.

## Robot Controller Setup

1. On the pendant, set the PLAY mode.
2. On the pendant, set remote ON.

The driver checks the two settings when it activates.

The ACU control group (`group_no`, default `0`) must have the same number of axes as the URDF has joints.

## Timing

- Cycle: 250 Hz (4 ms). The hardware runs in async slave mode.
- Command latency: approximately 40 ms (10 buffer slots × 4 ms).

## Startup

The full procedure is in [`workspaces/yaskawa/LAUNCH.md`](../../workspaces/yaskawa/LAUNCH.md).

1. Optional: start the zenoh router.
2. Monitor `/b_controlled_box_cm/activity`.
3. Start the controller manager `b_controlled_box_cm` on the ctrlX CORE.
4. Launch the bringup.

   ```bash
   ros2 launch yaskawa_acu_bringup yaskawa_acu_bringup.launch.xml
   ```

5. Set foreman to the `active` profile. Foreman activates the hardware and the controllers.

   ```bash
   ros2 action send_goal --feedback /foreman/set_profile foreman_msgs/action/SetProfile "{profile: 'active'}"
   ```

6. Launch MoveIt.

   ```bash
   ros2 launch yaskawa_acu_bringup yaskawa_acu_moveit.launch.xml
   ```

To test without a robot, use the mock hardware:

```bash
ros2 launch yaskawa_acu_bringup yaskawa_acu_bringup_mock.launch.xml
```

## Next Steps

The Yaskawa robot is now configured. Go to the [Commissioning PC Setup](../SETUP_COMMMISSIONING.md) to start the ROS 2 environment and control the robot.

## Troubleshooting

#### *The driver deactivates.*
If a command send fails or times out, the driver deactivates. Activate it again.

#### *The robot stops at its position, but there is no alarm.*
The ACU buffer is possibly empty. Then the robot stays at its position without an alarm, and the latency increases. Monitor `connection_health/buffer_estimated_spots`.
