# Next steps

**Congratulations on successfully commissioning your robot with b»controlled box!** 

Once you have basic movement established, you will likely want to start building complex applications. To extend the system, you have two primary approaches:

## 1. Extend existing container

You can take the existing commissioning container and bind your local workspace directories to it. This allows you to develop locally on your PC while running your nodes inside the pre-configured, network-ready Docker environment.

This can be configured in `docker-compose.yml`. To learn more about Docker volume binding, see: [docker volumes docs](https://docs.docker.com/reference/compose-file/volumes/)


## 2. Build from source in your workspace

If you prefer a native setup without Docker, you can use the `.repos` file in the `/src` directory of the container workspace. Once in the container, run:

```bash
rosds # ros-team-workspace alias for navigating to source directory of the ROS workspace
cat <robot-name>.jazzy.repos
```

You can use `vcs import` to pull all the necessary packages and build them directly from source in your own local workspace:

```bash
cd <your-workspace>/src
vcs import . < path-to/<robot-name>/jazzy.repos

```

Alternatively, you can simply copy the source file from the container itself.

# FAQ

### Can I use b»controlled box with ROS 2 Humble?

**Generally yes, but it is a bit complicated.** The b»controlled box builds `ros2_control` for **ROS 2 Jazzy**. Because of this, the main thing you have to take care of is the compatibility of ROS messages across your system network.

For your Humble workspace to communicate with the Jazzy `ros2_control` instance on the box, your local workspace must use the exact same message definitions.

The main offenders we identified would be `control_msgs` and `controller_manager_msgs`.

#### `control_msgs`

```yaml
control_msgs:
  type: git
  url: https://github.com/b-robotized-forks/control_msgs.git
  version: bCtrldBox/jazzy/gpio-controller-msgs
```
#### `controller_manager_msgs` (from `ros2_control` package - [link](https://github.com/b-robotized-forks/ros2_control/tree/bCtrldBox/jazzy/hw-unconfigured-cm-lifecycle/controller_manager_msgs))

```yaml
ros2_control:
  type: git
  url: https://github.com/b-robotized-forks/ros2_control.git
  version: bCtrldBox/jazzy/hw-unconfigured-cm-lifecycle
```

We currently use specific forks and branches for our control messages:

#### ⚠️ Work in progress!
This is still in testing. There might be some other statistics, navigation or other messages that break full compatibility with humble.


#### Our Recommendation:
Mixing ROS 2 distributions (especially across major changes in message structures) can cause MD5 hash and type mismatches over DDS/Zenoh, which are difficult to debug. It would be best long-term to transition your development workspace to ROS 2 Jazzy. This ensures seamless compatibility with the b»controlled box out of the, well, box.