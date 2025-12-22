# Kassow Robot Setup

To controller Kassow robot with b»controlled box, you will need install KORD CBun v3.0.1.
You can download it from the [KORD CBun release page here](https://gitlab.com/kassowrobots/kord-api/-/wikis/Master-CBun), copy it on an USB stick and plug it in the robot controller.
For details on installing CBun refer to the Kassow official documenation - nevertheless the process is also self explainatory - so you might just try by yourself 😃

The rest of the manual assumes that you have setup your ctrlX CORE to the [standard setup for running b»controlled box App explained here](https://github.com/b-robotized/b_ctrldbox_commissioning/blob/docs/kassow_port_guide/docs/SETUP_CTRLX.md).

Connect the Kassow Controller to the ctrlX CORE on the the X12 (CORE X7) or X51 (CORE X3) interface directly via ethernet cable.

## Kassow Controller Setup

Setup the IP address of the robot controller to be visible from the ctrlX CORE on the X12 (CORE X7) or X51 (CORE X3) interface.
Recommended IP address is *10.23.23.238*.
To change it do the following:

1. Choose `Workcell` in the left or right sidebar menu.
2. Choose `ethnet` under `INTERFACES` section and then `Options` in the sidebar menu.
3. Select `Static IP` and set parameters:
   `IP ADDRESS`: `10.23.23.238`
   `NETMASK`: `255.255.255.0`
4. And confirm by pressing `Re- Activate` button.

**Now you have to be able to ping the robot from the ctrlX CORE.**
(see Troubleshooting section for details)

## Activating CBun

1. Press the 3 dot icon in the upper right corner and then `CBuns`
  <p align="center">
  <img src="../assets/kassow/kassow_1.jpg" alt="Description of image" width="60%">
  </p>

2. When in CBuns tab choose the **right** `+` symbol to add CBun to the `Workcell`.

<p align="center">
<img src="../assets/kassow/kassow_2.jpg" alt="Description of image" width="60%">
</p>

3. Now select the CBun with the name `KORD` in the `Workcell` and press options in the left or right sidebar menu to see the settings and activation display of KORD CBun.
   **Make sure to set the port of the CBun to the correct value as defined by the URDF. Default value is *28283*.**

<p align="center">
<img src="../assets/kassow/kassow_3.jpg" alt="Description of image" width="60%">
</p>

4. Activate the KORD.

## Controlling the robot from your ROS 2 computer:

Make sure that you set the IP addresses and ROS 2 variables on your host computer as described in the [Commissioning PC Setup](../SETUP_COMMMISSIONING.md).

**THE KASSOW ROBOT IS NOT YET INTEGRATED IN THE COMMISSIONING CONTAINER - YOU WILL HAVE TO SETUP THE ROS 2 WORKSPACE LOCALLY AS DESCRIBED IN THE [kassow_kord_driver repository](https://github.com/b-robotized/kassow_kord_driver).**


## Troubleshooting

#### Connection issues when trying to set the robot to `inactive` state.
Make sure that the IP addresses are set correctly and you can ping the robot.
To ping it choose `Setting` » `Network Diagnostics` » `Ping` on the ctrlX CORE and enter the address of the robot controller in the `Address` filed.

#### *I changed the IP address of the CtrlX device, now I don't see ROS topics anymore on my commissioning PC/container*
Restart the ctrlX CORE.

#### *I can ping the CtrlX from my workspace and back, but I see no ROS topics.*
Ensure your IP address is `192.168.28.202` or `192.168.28.201`. These are the IPs the ROS network from b»controlled box sees.
Ensure you have exported `ROS_STATIC_PEERS="192.168.28.28` in you workspace terminal, to ensure the ROS network from your workspace sees the nodes from b»controlled box.
Ensure that you have the `ROS_DOMAIN_ID=0`.

Try restarting ROS 2 daemon with the following commands:
```
ros2 daemon stop
ros2 daemon start
ros2 topic list
```

**For other cases check the troubleshooting in [`kord_kassow_driver` repository](https://github.com/b-robotized/kassow_kord_driver?tab=readme-ov-file#troubleshooting).**

