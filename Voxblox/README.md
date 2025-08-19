# Voxblox Docker for vS-Graphs (ROS2 Jazzy)

This Docker environment provides a complete setup for [Voxblox](https://github.com/snt-arg/mav_voxblox_planning) and [vS-Graphs](https://github.com/snt-arg/visual_sgraphs). It is based on **Ubuntu 20.04**, with **ROS1 Noetic** (for running `Voxblox`) and **ROS2 Foxy** (for ROS1 to ROS2 bridging), and a custom tool called `vox2ros` that enables Voxblox messages to be translated and understood in **ROS2 Jazzy** of vS-Graphs.

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

## 📟 Available Commands

- Alias command `noetic` for sourcing **ROS1 Noetic**
- Alias command `foxy` for sourcing **ROS2 Foxy**
- Alias command `voxblox` for sourcing and running **Voxblox**:
```bash
source /root/voxblox_ws/ros1_ws/devel/setup.bash && roslaunch voxblox_skeleton skeletonize_map_vsgraphs.launch
```
- Alias command `bridge` for bridging **ROS1 Noetic** messages to **ROS2 Foxy**:
```bash
source /root/voxblox_ws/ros_bridge_ws/install/setup.bash && ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```
