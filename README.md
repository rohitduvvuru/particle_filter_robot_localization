# Particle Filter Robot Localization

Particle Filter Localization Video: 

<video src="kazam_dnuky2a0.movie.mp4" controls="controls" style="max-width: 100%;">
  Your browser does not support the video tag.
</video>

This repository contains a ROS Noetic package for known-map robot localization with a particle filter in Gazebo. The package launches a Triton robot in the TurtleBot3 house world, publishes odometry from Gazebo model state, runs a likelihood-field particle filter from laser scans, and visualizes the map, particles, estimated pose, laser data, and TF tree in RViz.

## Package Overview

The ROS package is named `cs603_particle_filter_d2` and is organized as follows:

```text
cs603_particle_filter_d2/
├── launch/                     # ROS launch files
├── maps/                       # Occupancy map and precomputed NumPy map data
├── models/triton/              # Triton Gazebo model assets
├── rviz/                       # RViz visualization configurations
├── scripts/                    # Python ROS nodes
├── CMakeLists.txt              # Catkin build configuration
└── package.xml                 # ROS package metadata and dependencies
```

Key nodes and assets:

- `scripts/particle_filter_d2.py` — particle filter localization node.
- `scripts/position_publisher.py` — publishes `/odom` from Gazebo's `triton` model state.
- `scripts/teleop_particle_filter.py` — keyboard teleoperation publisher for `/cmd_vel`.
- `launch/particle_filter_d2.launch` — starts Gazebo, the Triton robot, map server, TF publishers, particle filter, teleop, and RViz.
- `maps/house_map.yaml` and `maps/house_map.pgm` — occupancy-grid map used by `map_server`.
- `maps/metric_map.npy` and `maps/likelihood_field.npy` — precomputed map arrays used by the particle filter.

## Features

- Global particle initialization over free map cells.
- Odometry-based probabilistic motion update.
- LaserScan likelihood-field sensor model.
- Low-variance particle resampling.
- Random particle injection for robustness and recovery.
- RViz visualization of:
  - `/map`
  - `/scan`
  - `/pf/particles`
  - `/pf/estimated_pose`
  - TF frames

## Requirements

This project is intended for a ROS Noetic catkin workspace on Ubuntu.

Required ROS/system components include:

- ROS Noetic
- `catkin`
- `rospy`
- `geometry_msgs`
- `sensor_msgs`
- `nav_msgs`
- `std_msgs`
- `tf`
- `gazebo_ros`
- `gazebo_msgs`
- `map_server`
- `rviz`
- `turtlebot3_gazebo` for the TurtleBot3 house world

Required Python packages include:

- `numpy`
- `PyYAML`
- `pynput`

## Workspace Setup

Clone or copy this repository into the `src` directory of a catkin workspace:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone <repository-url> particle_filter_robot_localization
```

Build and source the workspace:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

If the scripts are not executable in your environment, enable execution with:

```bash
chmod +x src/particle_filter_robot_localization/cs603_particle_filter_d2/scripts/*.py
```

## Running the Demo

Launch the complete particle filter localization demo:

```bash
roslaunch cs603_particle_filter_d2 particle_filter_d2.launch
```

The launch file starts:

1. Gazebo with the TurtleBot3 house world.
2. The Triton robot model.
3. Static TF publishers for `map`, `world`, `odom`, `base_link`, and `base_scan` relationships.
4. `map_server` using `maps/house_map.yaml`.
5. `position_publisher.py` for `/odom`.
6. `particle_filter_d2.py` for localization.
7. `teleop_particle_filter.py` for keyboard control.
8. RViz with `rviz/particle_filter_d2.rviz`.

## Teleoperation Controls

When the teleop node is active, use the keyboard to move the robot:

| Key(s) | Action |
| --- | --- |
| `W` or Up Arrow | Move forward |
| `S` or Down Arrow | Move backward |
| `A` or Left Arrow | Strafe left |
| `D` or Right Arrow | Strafe right |
| `Q` | Rotate left |
| `E` | Rotate right |
| `X` | Increase linear speed |
| `Z` | Decrease linear speed |
| `Esc` | Stop teleop display; use `Ctrl+C` to exit |

## Particle Filter Parameters

The launch file exposes several arguments that can be overridden at runtime:

| Argument | Default | Description |
| --- | ---: | --- |
| `num_particles` | `1000` | Number of particles maintained by the filter. |
| `max_beams` | `20` | Maximum number of laser beams sampled per sensor update. |
| `z_hit` | `0.90` | Weight of the map-hit likelihood term. |
| `z_rand` | `0.10` | Weight of the random-measurement likelihood term. |
| `alpha1` | `0.02` | Rotation noise from rotation. |
| `alpha2` | `0.02` | Rotation noise from translation. |
| `alpha3` | `0.02` | Translation noise from translation. |
| `alpha4` | `0.02` | Translation noise from rotation. |
| `random_particle_ratio` | `0.10` | Fraction of particles randomly reinjected after each update. |
| `motion_threshold_trans` | `0.01` | Minimum translation before updating the filter. |
| `motion_threshold_rot` | `0.01` | Minimum rotation before updating the filter. |

Example override:

```bash
roslaunch cs603_particle_filter_d2 particle_filter_d2.launch num_particles:=1500 max_beams:=30 random_particle_ratio:=0.05
```

## ROS Interfaces

### Subscribed Topics

- `/odom` (`nav_msgs/Odometry`) — robot odometry from Gazebo.
- `/scan` (`sensor_msgs/LaserScan`) — laser scan data used by the sensor model.

### Published Topics

- `/pf/particles` (`geometry_msgs/PoseArray`) — current particle cloud.
- `/pf/estimated_pose` (`geometry_msgs/PoseStamped`) — estimated robot pose.
- `/odom` (`nav_msgs/Odometry`) — published by `position_publisher.py`.
- `/cmd_vel` (`geometry_msgs/Twist`) — published by `teleop_particle_filter.py`.

## Troubleshooting

- **`roslaunch` cannot find the package**: rebuild the workspace and run `source devel/setup.bash` from the workspace root.
- **Gazebo world cannot be found**: install or source the workspace containing `turtlebot3_gazebo`.
- **No particles appear in RViz**: confirm `/scan` and `/odom` are being published and that RViz is using the provided configuration.
- **Teleop does not respond**: ensure the terminal running the launch has keyboard focus and that `pynput` is installed.
- **Map or likelihood file errors**: verify the files in `cs603_particle_filter_d2/maps/` are present and readable.

## License

This package declares a BSD license in `package.xml`.
