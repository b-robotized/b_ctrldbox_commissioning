# KUKA Robot Setup

This guide covers the KUKA-specific steps for configuring the Robot Sensor Interface (RSI). This configuration is essential for enabling real-time communication between the robot controller and the b»Controlled Box

## 1. Configure the RSI Network Interface

You'll need to set up a dedicated network interface on the robot controller for RSI communication. The process varies slightly based on your KUKA System Software (KSS) version.

### For KSS >= 8.6 (KRC5):

1. On the teach pendant, navigate to `Start-up > Network configuration -> Add interface`.

2. Select the new entry and configure the following:

  * **Interface name:** rsi-interface (or similar).

  * **Address type:** Select Mixed IP address. This automatically creates the necessary real-time receive tasks.

  * **IP address**: Assign a static IP on a new subnet (b»Controlled Box [is configured for `10.23.23.5`](https://github.com/b-robotized/b_ctrldbox_commissioning/blob/575718f718cc3ad3302c491e38694cbc44a09ad0/kuka/KRC5/b_ctrldbox_rsi_eth.xml#L3)).

  * **Subnet mask:** `255.255.255.0.`

    Apply the changes and perform a **cold reboot** of the controller.

### For KSS >= 8.6 (KRC5):

1. Log in as **Expert** and minimize the **HMI** (`Start-up > Service > Minimize HMI`).

2. From the Windows Start Menu, run the **RSI-Network** utility.

3. Under "`RSI Ethernet`," select **New** and press **Edit**.

4. Enter a static IP address for the RSI ethernet (b»Controlled Box [is configured for `10.23.23.5`](https://github.com/b-robotized/b_ctrldbox_commissioning/blob/575718f718cc3ad3302c491e38694cbc44a09ad0/kuka/KRC5/b_ctrldbox_rsi_eth.xml#L3)).

5. Close the utility, maximize the HMI, and perform a **cold restart** with the **"Reload files"** option checked.


## 2. Prepare KRL Configuration Files

The **Kuka Robot Language** programs define the communication parameters. You must modify them to match your network setup before transferring them to the controller.

They can be found in the [kuka branch of `b_ctrldbox_commissioning`](https://github.com/b-robotized/b_ctrldbox_commissioning/tree/kuka-master) repository, and are present in the commissioning Docker Container under `~/commissioning/ros2_jazzy/src/b_ctrldbox_commissioning/kuka`

- `b_ctrldbox_rsi_eth.xml:`

  - Edit the `<IP_NUMBER>` tag to match the IP address of your commissioning host PC

- `b_ctrldbox_rsi.rsix:`

  - This file contains safety limits. The default values are typically sufficient to start.

  - Pay attention to the `<Timeout>` parameter. If you experience frequent disconnects, you may need to improve the real-time performance of your PC, for instance by using an RT-PREEMPT kernel.

- `b_ctrldbox_rsi.src:`

  - This file defines the robot's starting position. Adjust if necessary.

## 3. Transfer Files to Robot Controller

1. Copy the modified files to a USB drive.

2. Log in as Expert on the teach pendant.

3. Copy the files to the following directories:

  * b_ctrldbox_rsi.src -> `KRC:\R1\Program\`

  * All other files (`.xml`, `.rsix`) -> `C:\KRC\ROBOTER\Config\User\Common\SensorInterface\`

## 4. Verify network connection

Before proceeding, confirm that the b»Controlled Box can communicate with the robot's RSI interface.

1. In the **ctrlX CORE web UI**, navigate to `Settings -> Network Diagnostics`.

2. Ping the static IP you assigned to the robot's RSI interface.

***Note**: It is normal and expected to see replies marked as (DUP!). This indicates the RSI network task is active and responding correctly.*

## 5. Run the RSI Program

Finally, activate the RSI program on the robot.

1. On the teach pendant, select T1 mode.

2. Navigate to the `b_ctrldbox_rsi.src` program and **press the run/play button** while holding an enabling switch. The robot will move to its start position.

3. Press and hold the buttons again. A warning, `!!! Attention - Sensor correction goes active !!!`, will appear.

4. Confirm the warning. The program is now running and attempting to connect to the commissioning PC.

## Next Steps

The KUKA robot is now configured. Proceed to the [Commissioning PC Setup](../SETUP_COMMMISSIONING.md) to launch the ROS 2 environment and start controlling the robot.

# Troubleshooting

`test_joint_trajectory_controller.launch.xml` command fails - if the robot is not in the configured position. SHow how to reconfigure and rebuild the thing. Mention that in the future versions this will not be an issue