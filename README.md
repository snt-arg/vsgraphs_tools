# vS-Graphs Tools

This repository contains tools Docker setups for working with [vS-Graphs](https://github.com/snt-arg/visual_sgraphs). Each tool provides a pre-configured workspace with the necessary dependencies, `ROS` versions, and utilities.

## 📦 Available Tools

### I. 🦊 Voxblox Bridge Tool

It helps you to integrate `voxblox` in **vS-Graphs** for free-space cluster detection used in recognizing structural elements (e.g., rooms). The tool works by providing a tool (`vox2ros`) for **ROS1-Noetic (Voxblox) → ROS2-Foxy (Bridge) → ROS2-Jazzy (vS-Graphs)**. Read more about it in [this directory](/Voxblox/)

## 📎 Related Repositories

- 🔧 [vS-Graphs](https://github.com/snt-arg/visual_sgraphs)
- 🎞️ Scene Segmentor ([ROS2 Jazzy](https://github.com/snt-arg/scene_segment_ros))

## 📚 Citation

```bibtex
@article{tourani2025vsgraphs,
  title={vS-Graphs: Integrating Visual SLAM and Situational Graphs through Multi-level Scene Understanding},
  author={Tourani, Ali and Ejaz, Saad and Bavle, Hriday and Morilla-Cabello, David and Sanchez-Lopez, Jose Luis and Voos, Holger},
  journal={arXiv preprint arXiv:2503.01783},
  year={2025},
  doi={https://doi.org/10.48550/arXiv.2503.01783}
}
```

## 🔑 License

This project is licensed under the GPL-3.0 license - see the [LICENSE](/LICENSE) for more details.