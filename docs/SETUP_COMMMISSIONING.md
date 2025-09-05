# Commissioning PC setup

The commissioning PC host runs a Dockerized ROS 2 environment to communicate with the ctrlX CORE.

## 1. Prerequisites

### 1.1 Clone this repository

  ```
  git clone https://github.com/b-robotized/b_ctrldbox_commissioning.git
  cd b_ctrldbox_commissioning

  ```
### 1.2 Install Docker Desktop

  Docker desktop is required to run the container and set up networking via `docker compose` command.
  Refer to [this documentation](https://rtw.b-robotized.com/master/docker/general_information_docker/general_information_docker.html#installation-of-docker) for instructions.

### 1.3 Network Configuration:
  
  Configure a static IP address on your PC's Ethernet port that is on the 192.168.28.0/24 subnet (the default 192.168.28.28 with netmask 255.255.255.0).

  Verify you can ping the ctrlX controller at its IP address.

### 1.4 Environment Configuration:

  The repo contains an example `.env` file which must be configured with your host information for the container to properly run

  ```
  cp comissioning.env.example commissioning.env
  ```
  * `ROBOT_TYPE`: Set to your robot model (e.g., kuka).

  * `HOST_NETWORK_INTERFACE`: The name of the network interface on your PC connected to the ctrlX device (e.g., `eth0`). Use `ip addr` or `ifconfig` to find it.

  * `CONTAINER_MACVLAN_IP`: The static IP for the Docker container. The default (`192.168.28.29`) is usually fine.

3. ### 1.4 Make scripts executable:

  Make the `start/enter/stop` bash scripts executable:
  ```
  chmod +x start.sh enter.sh stop.sh
  ```

# 2. Usage

These are the commands for using the container

### 2.1 Start the container: `start.sh`

Upon first start, the container will be downloaded from our container repository. **It will require a login into our container registry!**
The container also creates the necessary network configuration on the host PC
```
./start.sh
```

### 2.2 Access the container shell: `enter.sh`

To access the container from other terminals, run:
```
./enter.sh
```
When entering the container, the ros environment will automatically be sourced. You can immediatelly run ros2 commands.


### 2.3 Verify container network:

Inside the container, run a quick ping to make sure it can see the ctrlX CORE:

```
ping 192.168.28.28
```

### 2.4 Launch the robot commands

To run scenario commands from the container, refer to `LAUNCH.md` for commands specific to your robot type.

Robot commands for a specific manufacturer can be found in this repo, [on the corresponding branch](https://github.com/b-robotized/b_ctrldbox_commissioning/branches) (e.g. `kuka-master` for [kuka](https://github.com/b-robotized/b_ctrldbox_commissioning/tree/kuka-master))


### 2.5 Stop the container: `stop.sh`

When you are finished, run this command from **host machine**, not the container.
```
./stop.sh
```


# Troubleshooting

What are some issues people will have?