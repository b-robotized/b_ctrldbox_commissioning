# UR Robot Setup

This guide covers the steps for configuring real-time communication with UR robot. In their case it is called `RTDE`.
This configuration is essential for enabling smooth and jerk free movement of the robot when using the b»controlled box.

First is important to know the **exact version** of robot control software.
You can check this under in the menu in info section.
There are two major versions (systems) used in UR robots *PolyScope 5* and *PolyScopeX*.
Each of them has also major and minor version that are important.

Before doing anything else, make sure that you have updated your software to the newest version.
The best place to find it, is to look for the `Release Notes` on the [*Support Resources* page](https://www.universal-robots.com/articles/) of UR.

After that follow the [official manual for the robot setup](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup.html) with `ur_client_library`.


# After configuring the robot

The UR robot is now configured. Proceed to the [Commissioning PC Setup](../SETUP_COMMMISSIONING.md) to launch the ROS 2 environment and start controlling the robot.


## Notes on using `URsim` with CtrlX

Here are some notes and guidelines for setting up **UR simulator** to control it with CtrlX.

### Install container:

```bash
curl -O https://raw.githubusercontent.com/UniversalRobots/Universal_Robots_Client_Library/refs/heads/master/scripts/start_ursim.sh
chmod +x start_ursim.sh
# -v 10+ for polyscope X, version 5+ for polyscope 5
./start_ursim.sh -m ur3e -v 10.8.0 
```

### Passwords:
Access the Polyscope X UI. If prompted, the default passwords are:

    operator

    easybot

    ursafe

### Configure UR sim

First, configure the URsim, either Polyscope5 or PolyscopeX as shown above.

### Docker network workaround

The tricky part of controlling URsim via CtrlX is connecting it through the Docker firewall, as it comes with several layers of isolation with regards to traffic coming from outside.

**The final goal is to be able to ping URsim container IP address from CtrlX.** Then all else should work.

Here are some notes that helped us, and might point you in the right direction:

> You only need to run this once per session. It applies system-wide but does not persist between reboots.

```bash
# 1. Force traffic to CtrlX out the physical port
# Tells the host that traffic destined for the CtrlX must exit the physical 
# ethernet interface immediately, bypassing the internal Docker bridge.
sudo ip route add <CTRL_IP>/32 dev <PHYSICAL_INTERFACE>

# 2. Enable routing and proxy ARP
# When CtrlX asks the network "who has <CONTAINER_IP>?", the container cannot 
# hear it. This allows the host machine to intercept the request, reply "I do!", 
# and route the traffic internally to the container.
sudo sysctl -w net.ipv4.ip_forward=1
sudo sysctl -w net.ipv4.conf.all.proxy_arp=1

# These rules below can be seen by running:
sudo iptables-save

# 3. Allow traffic into the container via DOCKER-USER chain
# Docker actively prevents external traffic from being forwarded into containers. 
# This inserts a rule to explicitly accept traffic heading toward the simulator.
sudo iptables -I DOCKER-USER 1 -d <CONTAINER_IP> -j ACCEPT

# 4. Remove Docker's strict RAW drop rules
# The container automatically adds strict DROP rules for packets destined for RTDE 
# ports that didn't originate from the Docker bridge. This deletes that blockade.
sudo iptables -t raw -D PREROUTING -d <CONTAINER_IP>/32 ! -i <DOCKER_BRIDGE> -j DROP
```
