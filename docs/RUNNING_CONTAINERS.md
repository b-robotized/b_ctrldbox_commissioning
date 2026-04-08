# Running Containers

This document provides a detailed reference for running the commissioning container in all supported scenarios. For the standard quick-start walkthrough, see [SETUP_COMMMISSIONING.md](SETUP_COMMMISSIONING.md).

> **This repository only runs containers.** If you want to build your own container image based on the pre-built containers, see the [FAQ & Next Steps](FAQ_NEXT_STEPS.md).

---

## Table of Contents

1. [Configuration Reference](#1-configuration-reference)
2. [Running a Public Image (Default)](#2-running-a-public-image-default)
3. [Running a Private Image](#3-running-a-private-image)
4. [Running a Locally-Built Custom Image](#4-running-a-locally-built-custom-image)
5. [Container Lifecycle Scripts](#5-container-lifecycle-scripts)
6. [Troubleshooting](#6-troubleshooting)

---

## 1. Configuration Reference

All configuration is done through a single `.env` file. Copy `comissioning.env.example` and rename it (e.g., `commissioning.env`). The scripts read the first `*.env` file found in the repository root.

| Variable | Required | Default | Description |
|---|---|---|---|
| `ROBOT_TYPE` | Yes | `ur` | Which robot image to use. Supported: `ur`, `kuka`, `kassow`, `pssbl`, `dobot`, `fanuc`. |
| `VERSION_TAG` | Yes | `1.6.x` | Image version tag. Should match your b-controlled-box app version. |
| `HOST_NETWORK_INTERFACE` | Yes | _(empty)_ | The network interface on your PC connected to the `192.168.28.x` network. Use `ip addr` to find it. |
| `CONTAINER_MACVLAN_IP` | Yes | `192.168.28.202` | Static IP assigned to the container on the macvlan network. |
| `CONTAINER_NAME` | No | _(auto)_ | Override the container name. Defaults to `b-robotized-<ROBOT_TYPE>-commission`. |
| `USE_PRIVATE_REGISTRY` | No | _(empty)_ | Set to `true` only if b-robotized has provided you with a private image. See [Section 3](#3-running-a-private-image). |

---

## 2. Running a Public Image (Default)

This is the standard workflow described in [SETUP_COMMMISSIONING.md](SETUP_COMMMISSIONING.md). Public images are available without authentication.

1. Configure your `.env` file with the correct `ROBOT_TYPE` and `VERSION_TAG`.
2. Run `./start.sh`.
   - On first run, the image is pulled from the public registry.
   - On subsequent runs, the locally cached image is used automatically.
3. Run `./enter.sh` to open a shell in the container.
4. Run `./stop.sh` when finished.

---

## 3. Running a Private Image

If b-robotized has built a custom image specifically for your project, it will be hosted on a private registry. You will have received:

- A **deploy token** for read-only access.
- The **`ROBOT_TYPE`** name to use (e.g., `my-client-poc`).

### Steps

1. In your `.env` file, set:
   ```
   ROBOT_TYPE=my-client-poc
   VERSION_TAG=1.6.x
   USE_PRIVATE_REGISTRY=true
   ```

2. Authenticate to the private registry (**one-time**, credentials are cached by Docker):
   ```bash
   echo "<deploy-token>" | docker login code.b-robotized.com:5050 -u ctrlx --password-stdin
   ```

3. Run `./start.sh` as normal. The image will be pulled from the private registry.

> **Note:** You only need to run `docker login` once. Docker caches the credentials in `~/.docker/config.json`. If the token expires, b-robotized will provide a new one.

---

## 4. Running a Locally-Built Custom Image

If you have built a custom image locally using the [b_controlled_box_commissioning_containers](https://github.com/b-robotized/b_controlled_box_commissioning_containers) repository, `start.sh` will automatically detect and use it without pulling from any remote registry.

### How it works 

When you run `build-robot-image.sh` in the `b_controlled_box_commissioning_containers` repository, the resulting image is tagged with the **full public registry path** locally:

```
code.b-robotized.com:5050/b_public/b_products/b_controlled_box/b-controlled-box-commissioning-containers/ur-commission:1.6.x
```

This is the same path that `start.sh` checks for. Since the image already exists locally, the pull step is skipped entirely.

### Steps

1. Build your custom image in the `b_controlled_box_commissioning_containers` repository:
   ```bash
   cd b_controlled_box_commissioning_containers/
   ./build-robot-image.sh my-custom-name my-tag
   ```

2. Return to this repository and make sure your `.env` has matching values:
   ```
   ROBOT_TYPE=my-custom-name
   VERSION_TAG=my-tag
   ```
   Leave `USE_PRIVATE_REGISTRY` empty (your local image uses the public registry tag).

3. Run `./start.sh` - it will find the local image and skip pulling.

> **To revert to the upstream image:** Remove the local image with `docker rmi <image-tag>` and run `./start.sh` again. It will pull the official version from the registry.

---

## 5. Container Lifecycle Scripts

| Script | Purpose |
|---|---|
| `./start.sh` | Pulls the image (if not local), tags it, and starts the container via `docker compose up -d`. |
| `./enter.sh` | Opens an interactive bash shell inside the running container. |
| `./stop.sh` | Stops and removes the container via `docker compose down`. |

The container is started in detached mode. You can open multiple terminals into the same container by running `./enter.sh` from separate terminal windows.

Inside the container, the ROS 2 environment is automatically sourced. You can immediately run `ros2` commands. For robot-specific launch instructions, see the `LAUNCH.md` file in the corresponding `workspaces/<robot>/` directory.

---

## 6. Troubleshooting

### "HOST_NETWORK_INTERFACE is not set"

You have not configured the `HOST_NETWORK_INTERFACE` variable in your `.env` file. Run `ip addr` on your host machine and look for the interface connected to the `192.168.28.x` subnet.

### "Image not found" / pull fails

- **Public image**: Check that `ROBOT_TYPE` and `VERSION_TAG` are correct. Verify your internet connection.
- **Private image**: Make sure `USE_PRIVATE_REGISTRY=true` is set and that you have run `docker login` with valid credentials.

### "unauthorized: access forbidden" during pull

You are trying to pull a private image without authentication. Run:
```bash
echo "<deploy-token>" | docker login code.b-robotized.com:5050 -u ctrlx --password-stdin
```

### Container starts but cannot ping ctrlX CORE

- Verify the `HOST_NETWORK_INTERFACE` is correct.
- Verify the `CONTAINER_MACVLAN_IP` is not already in use on the network.
- Check that your host PC can ping `192.168.28.7` (the ctrlX CORE default IP) directly.
- See [SETUP_CTRLX.md](SETUP_CTRLX.md) and [SETUP_NTP_SERVER.md](SETUP_NTP_SERVER.md) for detailed network configuration.

### I want to force a fresh pull of the image

Remove the cached local image first:
```bash
docker rmi <IMAGE_URL_FULL>
```
Then run `./start.sh` again. You can find the exact image URL by running `source ./common.sh && echo $IMAGE_URL_FULL`.
