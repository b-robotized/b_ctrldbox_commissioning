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

> **Note: ⚠️** In our case, the Zenoh router app on the **b»controlled box** listens, and the commissioning PC or container connects.
> The router listens only on one `XF10` address (default `192.168.28.28`) and on localhost, see [Zenoh router on the b»controlled box](#5-zenoh-router-on-the-bcontrolled-box).

To connect to the **b»controlled box**, on your commissioning PC, first run the router:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
# increase log level to see the connection happening. Try info, debug or trace.
export RUST_LOG=zenoh=warn,zenoh_transport=warn

# Connect to the XF10 address of your ctrlX device (default 192.168.28.28).
# Other addresses of the device, e.g. 10.x.x.x, do not answer on port 7447.
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

- **Connection refused on port `7447`?** Connect to the address the router listens on (see section 5), not to another address of the device.
  Check the router log in the logbook of the device: `Listening on XF10 address <address>`.

## 5. Zenoh router on the b»controlled box

The app `zenohd-router-jazzy-binary` (from 1.8.0) runs the router on the device.
Source and details: [`b_controlled_box_support_apps`](https://code.b-robotized.com/b-controlled-box/b_controlled_box_support_apps), README, section "Zenoh router".

### Files

All files are in the active configuration (`Settings > Solutions > Active configuration`, or `Manage app data` in SERVICE mode).
The apps create them with default values only when they do not exist; an update does not change existing files.

| File | Used by | Content |
|---|---|---|
| `zenoh-router/ZENOH_ROUTER_CONFIG.json5` | Zenoh router | Router configuration; `listen` endpoints |
| `zenoh-router/zenoh_router.env` | Zenoh router | `RUST_LOG`, `ZENOH_LISTEN_SUBNET`, `ZENOH_LISTEN_PREFERRED`, `ZENOH_LISTEN_WAIT_S` |
| `b-controlled-box/ZENOH_SESSION_CONFIG.json5` | bCtrldBox (RT app) | Session configuration: connects to `tcp/localhost:7447` |

`b-controlled-box/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5` is not used.

The configurations are the defaults of `rmw_zenoh_cpp` (`DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5`, `DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5`) of the pinned `rmw_zenoh` version.
In the router configuration, only the `listen` endpoints are changed.

### Listen addresses

```json5
endpoints: [
  "tcp/XF10_ADDRESS:7447",  // XF10 address for the ROS 2 PC, RViz, b»ui
  "tcp/127.0.0.1:7447",     // localhost IPv4: clients on the device, e.g. bCtrldBox
  "tcp/[::1]:7447"          // localhost IPv6: the same, if localhost resolves to IPv6
],
```

`XF10_ADDRESS` is a placeholder. At each start, the router app replaces it with:
1. `ZENOH_LISTEN_PREFERRED` (default `192.168.28.28`) if the device has this address,
2. else the first device address in `ZENOH_LISTEN_SUBNET` (default `192.168.28`), e.g. `192.168.28.13` when more devices are in one network,
3. else, after `ZENOH_LISTEN_WAIT_S` seconds (default 60), `ZENOH_LISTEN_PREFERRED`. The router then keeps running on localhost.

### Set your own address

Change the files, then restart the device.

- Fixed address: replace `XF10_ADDRESS` in `ZENOH_ROUTER_CONFIG.json5`, e.g. `"tcp/192.168.28.13:7447"`. The app then uses the file unchanged.
- Other subnet or preferred address: keep `XF10_ADDRESS` and change the variables in `zenoh_router.env`.

Files of router releases up to 1.0 listen on all addresses (`tcp/[::]:7447`).
After an update to 1.8.0, replace the `listen` endpoints with the ones above, or delete the file before the update.
