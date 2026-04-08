# Next steps

**Congratulations on successfully commissioning your robot with b-controlled box!**

Once you have basic movement established, you will likely want to start building complex applications. To extend the system, you have several approaches:

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

## 3. Build your own commissioning container

If you need to modify the commissioning container itself (e.g., add packages, integrate custom ROS 2 nodes, change build configurations), you can build your own image using the public container build repository:

**[b_controlled_box_commissioning_containers](https://github.com/b-robotized/b_controlled_box_commissioning_containers)**

That repository contains the Dockerfiles and build scripts for all publicly available commissioning images. See its README for instructions on building a custom image and using it with this repository.

Once you have built a custom image locally, `start.sh` will automatically detect and use it instead of pulling from the remote registry. See [RUNNING_CONTAINERS.md](RUNNING_CONTAINERS.md) for details on how this works.

# FAQ

## Can I use b»controlled box with ROS 2 `humble`?

**The b»controlled box app is decidedly `jazzy`,** due to several crucial technical details. One might make `humble` ros nodes talk to `foxy`, but there is an important
change done in `iron` (and subsequently, `humble`) that makes this near impossible (meaning, even if you do get it working, it will be a brittle nightmare to maintain).

Lets explain.

There are two main technical reasons:

### Type Description Distribution and Hashing (REP-2011) - breaks topics

Starting in `iron`, the system introduced [Type Hashes (REP-2011)](https://docs.ros.org/en/iron/Releases/Release-Iron-Irwini.html#type-description-distribution). This appends a unique hash (e.g., RIHS01_...) to the end of the topic type to ensure that both the publisher and the subscriber are using the *exact same message definition*.

- **Jazzy** (and newer): Automatically appends this hash to the topic type string at the middleware (RMW) level.

- **Humble** (and older): Predates this feature. It either uses the raw message name or fails to generate the hash (sometimes falling back to injecting the string `TypeHashNotSupported`).

Because the middleware routes messages by matching exact string paths for topics and types, a `jazzy` node and a `humble` node will see each other's topics as completely different types. They will simply ignore each other.

> This is confirmed in testing the `jazzy` b»controlled box app with `humble` workspace, and there is limited discovery of topics and **no transport of data** using either FastRTPS DDS or Zenoh as middleware.

---

### GID Storage Size Change - breaks services/actions

In `iron`, the [storage size for the Global Identifier (GID)](https://docs.ros.org/en/iron/Releases/Release-Iron-Irwini.html#change-the-gid-storage-to-16-bytes) was changed from 24 bytes to 16 bytes. This GID is used under the rmw hood to track Service and Action requests and route the response back to the correct client.

Because a `humble` node expects a 24-byte identifier and a `jazzy` node sends a 16-byte identifier, the memory alignment of the packets over the network is completely incompatible.

> Attempting to call a `jazzy` service from a `humble` client (or vice versa) results in corrupted data parsing at the RMW layer, leading to dropped calls or crashes.

---

### Ok, what do I do then?

These are not changes one can "hack" around, and are important architectural choices to advance ROS 2 as a framework. 

Our main suggestion is: **Standardize distribution to `jazzy`**. 

This is the simplest, easiest solution to keep you up to date, and enable you to use the **b»controlled box** application.
If developing on Ubuntu `22.04`, you can leverage Docker containers to standardize environment accross your projects - just build the container from Ubuntu `24.04`.

If you decide to upgrade to `jazzy` and need some support, don't hesitate to contact us for help!

### Further reading & references:

- [ROS Discourse: Incompatibility between distributions](https://discourse.openrobotics.org/t/incompatability-between-distributions/43747)

- [Robotics StackExchange: How to communicate between ROS2 Humble and ROS2 Jazzy seamlessly](https://robotics.stackexchange.com/questions/118150/how-to-communicate-between-ros2-humble-and-ros2-jazzy-seamlessly)

- [ROS 2 Iron Release Notes: Type Description Distribution](https://docs.ros.org/en/iron/Releases/Release-Iron-Irwini.html#type-description-distribution)

- [ROS 2 Iron Release Notes: Change the GID storage to 16 bytes](https://docs.ros.org/en/iron/Releases/Release-Iron-Irwini.html#change-the-gid-storage-to-16-bytes)
