# UR Robot Setup

## 1.

we need git@code.b-robotized.com:b-controlled-box/forks/Universal_Robots_ROS2_Driver.git
for launch files

git@github.com:UniversalRobots/Universal_Robots_Client_Library.git
for sim container

In container:
- power on the robot, start it
- hamburger menu - system - remote control - enable: this gives contro lto our ros2 hardware interface
- same menu - network - static address: here we set the 192.168.28.201, an address our ctrlx sees. The commissioning container is 192.168.28.202
- exit the menu, a new icon appears, says Local. Click on it and set to remote.
- now the robot is ready to receive commands from the CtrlX

- additionally, lead the robot to default home position to test JTC
positons: [0, -90, 0, -90, 0, 0]