# Commissioning Your Robot Environment

Welcome! This guide will walk you through setting up and running the ROS 2 environment for your robot running on `b-controlled box`

# Part 1: One-Time Host Machine Setup

You only need to do this the very first time you set up a computer.

1. ### Clone this repository

  ... and `cd` into it. It contains all the necessary config files to run all the robot variants we support

2. ### Configure Network:
  
  Set a static IP address on your machine within the `192.168.28.x` subnet (e.g., `192.168.28.10`), with a netmask of `255.255.255.0.`

  Verify you can ping the ctrlX controller at its IP address.

3. ### Configure the Environment:

  The repo contains a `.env` file which must be configured for the container to properly run.

  - `HOST_NETWORK_INTERFACE`: Network interface name on which you've connected to `b-controlled box`
    - via `ip addr` command, e.g. `enp0s8`
  - `DOCKER_IMAGE_API_KEY`: API key we've provided for the container repository
  - `CONTAINER_MACVLAN_IP`: static IP the container will have. 
    - take care to set it to `192.168.28.x` subnet!

3. ### Make scripts executable:

  Make the `start/enter/stop` bash scripts executable:
  ```
  chmod +x start.sh enter.sh stop.sh
  ```

#### That's it for setup!

# Part 2: Usage

These are the commands for using the container

- ### Start the container: `start.sh`

Upon first start, the container will be downloaded from our container repository. **It will require a login into our container registry!**

- ### Access the container shell: `enter.sh`

When accessing the container, the ros environment will automatically be sourced. You can immediatelly run ros2 commands.

- ### Verify network:

Inside the container, run a quick ping to make sure it can see the `b-controlled box` device

```
ping 192.168.28.202
```

- ### Launch the robot

To run scenario commands, `cd` into your specific robot config directory (for example, `kuka/KRC5`).

In there, refer to `LAUNCH.md` for commands on launching the scenario.

- ### Stop the container: `stop.sh`

When you are finished, run this command from **host machine** terminal.

INSTRUCTIONS.md
Prikaz stavke INSTRUCTIONS.md.