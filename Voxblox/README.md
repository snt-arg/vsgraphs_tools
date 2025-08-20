# 🦊 Voxblox Docker for vS-Graphs (ROS2 Jazzy)

This Docker environment provides a complete setup for integrating [Voxblox](https://github.com/snt-arg/mav_voxblox_planning) in [vS-Graphs](https://github.com/snt-arg/visual_sgraphs). It is based on **Ubuntu 20.04**, with **ROS1 Noetic** (for running `Voxblox`) and **ROS2 Foxy** (for ROS1 to ROS2 bridging), and a custom tool called `vox2ros` that enables Voxblox messages to be translated and understood in **ROS2 Jazzy** of vS-Graphs.

## 🚀 Getting Started

I. Build the **Dockerfile** and enter the environment's terminal
```bash
cd [path]/vsgraphs_tools/Voxblox/docker
docker compose build

docker compose up -d
docker exec -it voxblox_bridge bash
```

II. Run `mprocs` for running multiple processes in one terminal. The configuration file is available [here](/Voxblox/mprocs.yml)

```bash
mprocs
```

Then you can start running `voxblox`, `bridge`, and the `vox2ros` tool (in the same order). The `vox2ros` tool waits to get connected to the tunnel started in **vS-Graphs**. The documentation can be found in [the dedicated page](https://github.com/snt-arg/visual_sgraphs/blob/master/doc/INSTALLATION.md).

## 📟 Available Commands

- Alias command `noetic` for sourcing **ROS1 Noetic**
- Alias command `foxy` for sourcing **ROS2 Foxy**
- Alias command `mprocs` for running **Mprocs**
- Alias command `voxblox` for sourcing and running **Voxblox**:
```bash
source /root/voxblox_ws/ros1_ws/devel/setup.bash && roslaunch voxblox_skeleton skeletonize_map_vsgraphs.launch
```