# ROS 2 Communication with Zenoh (rmw_zenoh_cpp)

This guide explains how to configure and use Zenoh as the ROS 2 middleware (`rmw_zenoh_cpp`) for communication between the **b»controlled box** system, your development PC, and other devices on the network.

## 1. Overview and Concepts

When using Zenoh with ROS 2, the network architecture is slightly different from traditional DDS:

**Router (`zenohd`):**  
Think of this as a `ros2 daemon` running on your host. It handles network discovery and host-to-host communication. Typically, you run one router per host. Routers must be configured to talk to each other across the network.

**Sessions:**  
Every ROS 2 node creates a Zenoh "session" that communicates through the local loopback interface (`lo`). The router then picks up this traffic and handles the complex host-to-host routing.

The router is required for the initial discovery phase. In the common case, even if the router process dies, the established connection between ROS 2 nodes will persist, though new nodes will not be able to discover each other until the router is restarted.

## 2. Configuring `zenoh`

First, make sure you have `ros-<distro>-rmw-zenoh-cpp` package installed. in your workspace.

Secondly, stop your `ros2 daemon`. This is a DDS daemon used for ros2 CLI which we don't need, as we're running our own "daemon" in the form of a `zenoh` router.

```bash
ros2 daemon stop
```

By default, Zenoh uses configuration files, but we can easily override these settings using the `ZENOH_CONFIG_OVERRIDE` environment variable.

The format is:

```bash
export ZENOH_CONFIG_OVERRIDE="key/path/to/field1=value1;key/path/to/field2=value2"
```

Because Zenoh operates over TCP, you must configure one router to listen for connections, and the other router to reach out and connect.

> **Note: ⚠️** In our case, **b»controlled box** is configured to listen on all interfaces, and the commissioning container is configured to connect. This is done via a separate snap package running the `zenoh` router, which is provided with the app.

To connect to the **b»controlled box**, on your commissioning PC, first run the router:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
# increase log level to see the connection happening. Try info, debug or trace.
export RUST_LOG=zenoh=warn,zenoh_transport=warn

# Connect to your CtrlX device IP
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/192.168.28.28:7447"]'

# Start the router and leave it running.
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Then, in all other terminals, make sure you have these variables exported. Either add them to `.bashrc` or to `install/setup.sh` for auto-sourcing.
```bash
export RUST_LOG=zenoh=warn,zenoh_transport=warn
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

> **Note:** ⚠️currently, by default, all `ros2` CLI calls spawn a background `ros2 daemon` automatically. To prevent this, we can call them with `--no-daemon` option. However, this gets tedious really quickly. Look at the snippet below for your `.bashrc` or `install/setup.sh` for automatic appending of the `--no-daemon` option:

<details>
  <summary>Snippet for automatic append.</summary>
  
  **Trigger Conditions:**

  * The environment variable ``RMW_IMPLEMENTATION`` is set to ``rmw_zenoh_cpp``.
  * OR the environment variable ``RTW_NO_DAEMON`` is set to ``1``.
  * AND the user hasn't already passed the ``--no-daemon`` flag manually in the command.

  ```bash

    function ros2() {
    local base_cmd="ros2"
    local should_append=false

    if [[ "$RMW_IMPLEMENTATION" == "rmw_zenoh_cpp" ]]; then
      should_append=true
    fi

    if [[ "$RTW_NO_DAEMON" == "1" ]]; then
      should_append=true
    fi

    # If the command already has --no-daemon, skip
    if [[ "$*" == *"--no-daemon"* ]]; then
      command $base_cmd "$@"
      return
    fi

    # if we don't append, run normal ros2
    if ! $should_append; then
      command $base_cmd "$@"
      return
    fi

    # commands that actually support the --no-daemon flag
    local daemon_cmds=("topic" "node" "service" "param" "interface" "lifecycle")
    local is_daemon_cmd=false

    for cmd in "${daemon_cmds[@]}"; do
      if [[ "$1" == "$cmd" ]]; then
        is_daemon_cmd=true
        break
      fi
    done

    if $is_daemon_cmd; then
      command $base_cmd "$@" --no-daemon
    else
      command $base_cmd "$@"
    fi
  }
  
  ```
</details>

## 3. Using `zenoh`

If you're using **b»controlled box** commissioning container, all of this configuration is already set up.

From here on, ensure in one terminal that you have `ros2 run rmw_zenoh_cpp rmw_zenohd` router running, and use your `ros2 topic list` and other commands normally.

## 4. Troubleshooting

- **No topics visible?** Ensure both the router (`rmw_zenohd`) and the application terminals have export `RMW_IMPLEMENTATION=rmw_zenoh_cpp` set.

- **Firewall issues?** Verify that TCP port `7447` is open and allowed through your host's firewall (e.g., `sudo ufw allow 7447/tcp`).

- **Router logs** Keep the rmw_zenohd terminal visible. The `RUST_LOG=zenoh=debug,zenoh_transport=debug` environment variable will print detailed diagnostic information if the TCP handshake is failing.

- **CtrlX zenoh router app active?** Ensure the `zenoh` router app is installed and active on your `CtrlX` device.