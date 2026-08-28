# Franka Robot Setup

To control the Franka Robot with b»controlled box, you will need to follow these steps:

1. Login to the Franka Desk Web UI, which can be accessed by this url [https://10.50.50.1](https://10.50.50.1)
![Franka Self Test Acknowledge](../assets/franka/franka_login_screen.png)

2. Then enter the credentials to login
![Franka Desk Web UI](../assets/franka/franka_desk_web_ui.png)

3. After that, on the Desk tab, select on the Exec mode

![Franka Exec Mode](../assets/franka/modes_exec_prog.png)

4. Then toggle unlock switch, and click on Confirm. It will take a moment to unlock.

![Franka toggle Unlock](../assets/franka/unlock_joints.png)

![Franka Joints Unlocked](../assets/franka/unlocking_joints.png)

5. Once, the joints are unlocked, then select **Franka-Bota** (Your Franka Robot name) on top right corner.

![Franka to activate FCI](../assets/franka/to_activate_fci.png)

5. Then, you click activate FCI.

![Franka activate FCI](../assets/franka/click_for_activate_fci.png)

6. Now, a pop up message will show to activate FCI.

![Franka confirm FCI](../assets/franka/fci_activate_confimation.png)

7. Once, the Franka is activated the dashboard will look like this.

![Franka FCI activated](../assets/franka/franka_dashboard_fci_activated.png)

8. To deactivate FCI, select the Robot name from the top right corner and click on Deactivate FCI.

![Franka FCI deactivate](../assets/franka/deactivate_fci.png)


>[!NOTE]
> We also provide the ros2 lifecycle node which can be used to lock and unlock the Franka robot without using the web desk user interface and to run these nodes you can follow the following commands, which are as follows: 

#### Commands for ROS2 Lifecycle Nodes
```bash
# start the ros2 lifecylce node
ros2 launch franka_lock_unlock franka_lock_unlock.launch.xml hostname:=<hostname> username:=<username> password:=<pwd>

# configure the node (it will take 2 mins to configure, as it must run self-test)
ros2 lifecycle set /franka_lock_unlock_node configure

# activate (unlock robot and enable FCI)
ros2 lifecycle set /franka_lock_unlock_node activate

# deactivate (lock robot)
ros2 lifecycle set /franka_lock_unlock_node deactivate

# cleanup
ros2 lifecycle set /franka_lock_unlock_node cleanup 

# shutdown 
ros2 lifecycle set /franka_lock_unlock_node shutdown 
```
## Next Steps

The Fanka robot is now unlocked and ready to use. Proceed to the [Commissioning PC Setup](../SETUP_COMMMISSIONING.md) to launch the ROS 2 environment and start controlling the robot.

## Troubleshooting

### Emergency Stop

When the emergency button is pressed, the Franka robot gets locked and the hardware interface gets into the unconfigured state. Also all other controllers like Franca joint trajectory controller and joint state publisher get to an inactive state. To recover this, you can run the following steps:

1. Open the Franka Desk Web UI if you are not using ROS2 lifecycle node, then from the Exec Mode, toggle the Unlock option, and wait for it to unlock the robot. Once, the robot is unlocked, then need to activate FCI (Franka Control Interface). However, if you are using Franka Lock Unlock Lifecycle Node, then run these commands.

```bash
# deactivate (lock robot)
ros2 lifecycle set /franka_lock_unlock_node deactivate

# activate (unlock robot and enable FCI)
ros2 lifecycle set /franka_lock_unlock_node activate
```

2. Run foreman commands to switch the Franka Hardware Interface and ROS2 controllers to inactive and then active state.
```bash
ros2 service call /foreman/set_goal foreman_msgs/srv/SetGoal "{goal: 'inactive'}"

ros2 service call /foreman/set_goal foreman_msgs/srv/SetGoal "{goal: 'active'}"
```

3. After this, you are ready to use the Robot normally.


### Franka Self Test

Franka robot is required to run self test within a 24-hour interval. If the Franca robot has not been tested then it will unlock the robot until you acknowledge the self test and run this.

To resolve this, you have to go to the Franca web user interface as shown in the below image. There you can basically click on **Acknowledge and execute** and then wait for the Franca to run the self test. Once the self test is finished you can follow the instructions to unlock the robot and activate the Franca controller interface. 

![Franka Self Test Acknowledge](../assets/franka/acknowledge_self_test.png)

Then, Franka will reboot and execute self test. You need to wait for 2 mins, before using it again.

![Franka Self Test Acknowledge](../assets/franka/frank_self_test_rebooting.png)

>[!NOTE]
> However if you are using the Franca lock unlock lifecycle node, you don't have to worry about this issue as we have a timer inbuilt which will execute this self test once within 24 hours. 

Once you unlock the franka robot, then you can run these commands to inactivate the controllers and hardware interface, and then activate them again using the foreman.

```bash
ros2 service call /foreman/set_goal foreman_msgs/srv/SetGoal "{goal: 'inactive'}"

ros2 service call /foreman/set_goal foreman_msgs/srv/SetGoal "{goal: 'active'}"
````

### Franka gets locked

If Franka gets locked then the Franka hardware interface will go to un-configured and ros2 controllers will also go to `inactive`. To recover from this, you must go to the Franka desk web UI (if you are not using Franka lock unlock lifecycle node) and control the robot manually and then activate the FCI by following the steps shown in the top instructions. Once this is done you need to run the following foreman commands to run the state from inactive to active.
