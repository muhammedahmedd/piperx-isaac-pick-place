# Piper X Pick-and-Place

This repository contains a ROS 2 Humble pick-and-place project for the Piper X arm using MoveIt 2, Isaac Sim, ArUco marker detection, and a gripper TCP frame.

The project is designed to be used as the root of a ROS 2 workspace. The main project packages live inside `src/`, while external dependencies such as the Piper X simulation/MoveIt fork are cloned into `src/` separately.

## Repository structure

```text
piperx_ws/
├── Dockerfile
├── compose.yaml
├── ros_entrypoint.sh
├── src/
│   ├── piperx_control/
│   ├── piperx_perception/
│   └── agx_arm_sim/              # cloned separately
└── README.md
```

## External dependencies

This project uses a Piper X-focused fork of `agx_arm_sim`. It should be cloned into the workspace `src/` folder:

```bash
cd ~/piperx_ws
git clone --recursive https://github.com/muhammedahmedd/piperx_arm_sim.git src/agx_arm_sim
```

The `piperx_arm_sim` repository includes the Piper X MoveIt configuration and points to the Piper X URDF fork with the added `gripper_tcp` frame.

## Docker setup

This repository includes a Docker setup for the ROS 2 Humble and MoveIt 2 environment.

Build the Docker image:

```bash
cd ~/piperx_ws
docker compose build
```

Start the container:

```bash
docker compose up -d
```

Enter the container:

```bash
docker exec -it ros2_humble bash
```

The workspace is mounted into the container at:

```text
/workspace/piperx_ws
```

If your workspace is not located at `~/piperx_ws`, update the volume path in `compose.yaml`.

## Build the ROS 2 workspace

Inside the container:

```bash
cd /workspace/piperx_ws
colcon build
source install/setup.bash
```

## Run MoveIt

Start the Isaac Sim scene first. After Isaac Sim is running, launch MoveIt and RViz:

```bash
ros2 launch piper_x_gripper_moveit_config demo.launch.py
```

The Piper X MoveIt configuration in `piperx_arm_sim` already includes the Isaac Sim topic-based ROS 2 control setup, so users do not need to manually modify the MoveIt hardware configuration.

## Notes

Generated ROS folders such as `build/`, `install/`, and `log/` are ignored because they are created locally by `colcon build`.

The Docker image provides the ROS 2 and MoveIt environment. The project workspace is mounted into the container so code changes can be made on the host machine and built inside the container.
