# Using This Repository

This repository is the **operational hub** for running the b-Controlled Box commissioning environment. It contains the Docker Compose configuration, lifecycle scripts, and per-robot workspace files needed to start, use, and stop the commissioning container.

> **You do not build images here.** This repository only _pulls_ and _runs_ pre-built container images. If you want to build a custom image, see the [releases](https://github.com/b-robotized/b-controlled-box-commissioning-containers) repository.

---

## Table of Contents

1. [Prerequisites](#1-prerequisites)
2. [Quick Start](#2-quick-start)
3. [Configuration Reference](#3-configuration-reference)
4. [Workflow A -- Running a Public Image (Default)](#4-workflow-a----running-a-public-image-default)
5. [Workflow B -- Running a Private Image](#5-workflow-b----running-a-private-image)
6. [Workflow C -- Running a Locally-Built Custom Image](#6-workflow-c----running-a-locally-built-custom-image)
7. [Container Lifecycle](#7-container-lifecycle)
8. [Troubleshooting](#8-troubleshooting)

---

## 1. Prerequisites

- **Docker** must be installed. See [Docker installation guide](https://rtw.b-robotized.com/master/operating_system/docker/general_information_docker/general_information_docker.html#installation-of-docker).
- **Network**: Your PC must have a network interface connected to the `192.168.28.x` subnet where the ctrlX CORE device lives. See [SETUP_CTRLX.md](docs/SETUP_CTRLX.md) for details.
- Clone this repository:
  ```bash
  git clone https://github.com/b-robotized/b_ctrldbox_commissioning.git
  cd b_ctrldbox_commissioning
  ```

---

## 2. Quick Start

```bash
# 1. Create your environment file from the template
cp comissioning.env.example commissioning.env

# 2. Edit the .env file -- set ROBOT_TYPE, VERSION_TAG, and HOST_NETWORK_INTERFACE
#    (see Section 3 for a full reference of all variables)

# 3. Make the scripts executable (only needed once)
chmod +x start.sh enter.sh stop.sh

# 4. Start the container (pulls the image on first run)
./start.sh

# 5. Open a shell inside the running container
./enter.sh

# 6. When finished, stop the container
./stop.sh
```

---

## 3. Configuration Reference

All configuration is done through a single `.env` file. Copy `comissioning.env.example` and rename it (e.g., `commissioning.env`). The scripts read the first `*.env` file found in this directory.

| Variable | Required | Default | Description |
|---|---|---|---|
| `ROBOT_TYPE` | Yes | `ur` | Which robot image to use. Supported: `ur`, `kuka`, `kassow`, `pssbl`, `dobot`, `fanuc`. |
| `VERSION_TAG` | Yes | `1.6.x` | Image version tag. Should match your b-controlled-box app version. |
| `HOST_NETWORK_INTERFACE` | Yes | _(empty)_ | The network interface on your PC connected to the `192.168.28.x` network. Use `ip addr` to find it. |
| `CONTAINER_MACVLAN_IP` | Yes | `192.168.28.202` | Static IP assigned to the container on the macvlan network. |
| `CONTAINER_NAME` | No | _(auto)_ | Override the container name. Defaults to `b-robotized-<ROBOT_TYPE>-commission`. |
| `USE_PRIVATE_REGISTRY` | No | _(empty)_ | Set to `true` only if b-robotized has provided you with a private image. See [Workflow B](#5-workflow-b----running-a-private-image). |

---

## 4. Workflow A -- Running a Public Image (Default)

This is the standard workflow. Public images are available without authentication.

1. Configure your `.env` file with the correct `ROBOT_TYPE` and `VERSION_TAG`.
2. Run `./start.sh`.
   - On first run, the image is pulled from the public registry.
   - On subsequent runs, the locally cached image is used automatically.
3. Run `./enter.sh` to open a shell in the container.
4. Run `./stop.sh` when finished.

**What happens under the hood:**

```
start.sh
  -> common.sh resolves IMAGE_URL_FULL to the public registry
  -> checks if image exists locally
  -> if not: docker pull <public-registry>/<robot>-commission:<tag>
  -> docker tag <full-url> b-robotized/<robot>-commission:<tag>
  -> docker compose up -d (uses the re-tagged local image)
```

---

## 5. Workflow B -- Running a Private Image

If b-robotized has built a custom image specifically for your project, it will be hosted on a private registry. You will have received:

- A **deploy token** (username + password) for read-only access.
- The **`ROBOT_TYPE`** name to use (e.g., `pssbl-poc`).

### Steps

1. In your `.env` file, set:
   ```
   ROBOT_TYPE=pssbl-poc
   VERSION_TAG=1.6.x
   USE_PRIVATE_REGISTRY=true
   ```

2. Authenticate to the private registry (**one-time**, credentials are cached by Docker):
   ```bash
   echo "<b-robotized-access-token>" | docker login code.b-robotized.com:5050 -u ctrlx --password-stdin
   ```
   Enter the deploy token username and password when prompted.

3. Run `./start.sh` as normal. The image will be pulled from the private registry.

> **Note:** You only need to run `docker login` once. Docker caches the credentials in `~/.docker/config.json`. If the token expires, b-robotized will provide a new one.

---

## 6. Workflow C -- Running a Locally-Built Custom Image

If you have built a custom image locally using the [releases](https://github.com/b-robotized/b-controlled-box-commissioning-containers) repository, `start.sh` will automatically detect and use it without pulling from any remote registry.

### How it works

When you run `build-robot-image.sh` in the `releases` repository, the resulting image is tagged with the **full public registry path** locally:

```
code.b-robotized.com:5050/b_public/.../ur-commission:1.6.x
```

This is the same path that `start.sh` checks for. Since the image already exists locally, the pull step is skipped entirely.

### Steps

1. Build your custom image in the `releases` repository:
   ```bash
   cd releases/
   ./build-robot-image.sh ur 1.6.x
   ```

2. Return to this repository and make sure your `.env` has matching values:
   ```
   ROBOT_TYPE=ur
   VERSION_TAG=1.6.x
   ```
   Leave `USE_PRIVATE_REGISTRY` empty (your local image uses the public registry tag).

3. Run `./start.sh` -- it will find the local image and skip pulling.

> **To revert to the upstream image:** Remove the local image with `docker rmi <image-tag>` and run `./start.sh` again. It will pull the official version from the registry.

---

## 7. Container Lifecycle

| Script | Purpose |
|---|---|
| `./start.sh` | Pulls the image (if not local), tags it, and starts the container via `docker compose up -d`. |
| `./enter.sh` | Opens an interactive bash shell inside the running container. |
| `./stop.sh` | Stops and removes the container via `docker compose down`. |

The container is started in detached mode. You can open multiple terminals into the same container by running `./enter.sh` from separate terminal windows.

Inside the container, the ROS 2 environment is automatically sourced. You can immediately run `ros2` commands. For robot-specific launch instructions, see the `LAUNCH.md` file in the corresponding `workspaces/<robot>/` directory.

---

## 8. Troubleshooting

### "HOST_NETWORK_INTERFACE is not set"

You have not configured the `HOST_NETWORK_INTERFACE` variable in your `.env` file. Run `ip addr` on your host machine and look for the interface connected to the `192.168.28.x` subnet.

### "Image not found" / pull fails

- **Public image**: Check that `ROBOT_TYPE` and `VERSION_TAG` are correct. Verify your internet connection.
- **Private image**: Make sure `USE_PRIVATE_REGISTRY=true` is set and that you have run `docker login code.b-robotized.com:5050` with valid credentials.
- **Custom image**: Make sure you built the image with matching `ROBOT_TYPE` and `VERSION_TAG` values in the `releases` repository.

### "unauthorized: access forbidden" during pull

You are trying to pull a private image without authentication. Run:
```bash
echo "<b-robotized-access-token>" | docker login code.b-robotized.com:5050 -u ctrlx --password-stdin
```
and enter your deploy token credentials.

### Container starts but cannot ping ctrlX CORE

- Verify the `HOST_NETWORK_INTERFACE` is correct.
- Verify the `CONTAINER_MACVLAN_IP` is not already in use on the network.
- Check that your host PC can ping `192.168.28.7` (the ctrlX CORE default IP) directly.
- See [SETUP_CTRLX.md](docs/SETUP_CTRLX.md) and [SETUP_NTP_SERVER.md](docs/SETUP_NTP_SERVER.md) for detailed network configuration.

### I want to force a fresh pull of the image

Remove the cached local image first:
```bash
docker rmi <IMAGE_URL_FULL>
```
Then run `./start.sh` again. You can find the exact image URL by running `source ./common.sh && echo $IMAGE_URL_FULL`.
