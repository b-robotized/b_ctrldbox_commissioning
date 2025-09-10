# ctrlX CORE device setup

For help with issues, refer to [Troubleshooting](#troubleshooting) at the end of this file.

## 1. Network Connection

1. Power on the ctrlX CORE device.

2. Connect your PC to the device's commissioning port (typically `XF10`) via an Ethernet cable.

3. The device has a default static IP: `192.168.28.28`. Open a web browser and navigate to this address. You should see the ctrlX CORE login screen.


![login-screen](assets/login.png)

Tip: For a general introduction to the ctrlX OS interface, see the official [Rexroth ctrlX OS Getting Started Playlist.](https://www.youtube.com/watch?v=kV8TtBNVecM&list=PLRO3LeFQeLyN8HlLG8VZzgoCLqTUcTPUx)


## 2. Configure Robot Network Interface

Ensure the robot is powered on and connected. Then:

1. In the ctrlX UI, navigate to `Settings > Network Interfaces.`

2. Select the Ethernet port connected to your robot.

3. Go to the `IPv4` tab and set a static IP address that is on the same subnet as your robot.

By default, we use:
#### `XF10: 192.168.28.28` for connection with PC
![network-interface10](assets/network_interface_XF10.png)

#### `XF12: 10.23.23.28` for connection with robot
![network-interface12](assets/network_interface_XF12.png)

4. To validate the connection, navigate to `Settings > Network Diagnostics` and ping your robot.

![ping](assets/ctrlx_ping_robot.png)

## 3. Install the Control Bundle App

1. Obtain the b»Controlled Box Snap application file. If you don't have it, you can find apps on the [Rexroth Store.](https://community.boschrexroth.com/ctrlx-os-store-apps-oc2pqqwn)

2. Ensure the device is in **SERVICE** mode. This mode allows for app installation but does not provide real-time guarantees.

3. Navigate to `Settings > Apps.`

4. Upload the Snap file and click `Install`. The app will start automatically.

## 4. Understanding Operation Modes

The ctrlX OS has three key modes that manage the lifecycle of real-time applications.

b»Controlled Box controller manager responds to these modes to ensure safe operation. For example, switch **OPERATIONAL** -> **SETUP** will automatically deactivate controllers that send motion commands.

![modes](assets/modes.png)

* **SERVICE**: The default mode for system configuration. You can install, update, or remove apps. Real-time execution is disabled.
  * controller manager waits for robot description

* **SETUP**: A real-time guaranteed mode for setting up the robot scenario.
  * all hardware interfaces can be active. Controllers that do not claim command interfaces can be active

* **OPERATIONAL**: The full real-time production mode. 
  * all hardware interfaces and controllers can be active and the system is ready to control the robot.



## 5. Configure the Real-Time Scheduler

**The scheduler** is the heart of the ctrlX CORE's real-time system. It executes tasks from applications, called "callables," on a deterministic, real-time thread. b»Controlled Box ROS 2 controller manager's `read/update/write` loop is executed within such a callable.

You can configure the cycle time of this callable to match your robot's communication and control requirements.

1. Navigate to `Settings -> Scheduler -> Configuration` and select the **Edit** action in the Action column (the pencil icon)
2. Adjust the cycle time and save.

![scheduler](assets/scheduler.png)

Important: Any changes made to the scheduler settings will only be applied to the control bundle after the device transitions from SERVICE mode to SETUP mode.


## 6. Add app License - TBA

For now, official licensing is to be implemented

# Troubleshooting

### Restarting the ctrlX CORE
To restart the device, navigate to `Settings -> Shutdown` and select **Restart**