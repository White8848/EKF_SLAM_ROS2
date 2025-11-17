# EKF SLAM Package

This package implements Extended Kalman Filter (EKF) based SLAM for TurtleBot3 in a Gazebo bookstore environment.

## Package Structure

```
ekf_slam/
├── ekf_slam/              # Python package
│   ├── ekf_slam_node.py   # EKF SLAM implementation
│   └── __init__.py
├── launch/                # Launch files
│   ├── robot_bookstore.launch.py   # Gazebo simulation + TF publishers
│   ├── ekf_slam.launch.py          # EKF SLAM node
│   └── view_robot.launch.py        # RViz visualization
├── rviz/                  # RViz configurations
│   └── robot_view.rviz    # Display config for SLAM & navigation
├── worlds/                # Gazebo world files
│   └── bookstore/         # Bookstore world and models
└── README.md
```

## Demo

![EKF SLAM + Nav2 Demo](doc/demo.gif)

*EKF SLAM mapping and Nav2 autonomous navigation demonstration in the bookstore environment*

## Prerequisites

- ROS 2 (Humble/Jazzy)
- Gazebo Classic
- TurtleBot3 packages
- Nav2 (optional, for navigation)

## Installation

```bash
cd ~/ekf_slam_ws/src
git clone https://github.com/White8848/EKF_SLAM_ROS2.git ekf_slam
cd ~/ekf_slam_ws
colcon build --packages-select ekf_slam
source install/setup.bash
```

## Usage

### 1. Launch Simulation Environment

Start Gazebo with bookstore world and robot TF publishers:

```bash
ros2 launch ekf_slam robot_bookstore.launch.py
```

This will:
- Load the bookstore world in Gazebo
- Spawn TurtleBot3 Waffle Pi robot
- Publish static TF transforms (base_footprint → base_link → base_scan/imu_link/camera_*)

### 2. Run EKF SLAM Node

In a new terminal, start the EKF SLAM algorithm:

```bash
ros2 launch ekf_slam ekf_slam.launch.py
```

This node will:
- Subscribe to `/scan` (laser data) and `/odom` (odometry)
- Build an occupancy grid map (`/map` topic)
- Publish estimated robot pose (`/ekf_pose`)
- Broadcast `map → odom` TF transform

### 3. Visualize in RViz

In another terminal, launch RViz with pre-configured displays:

```bash
ros2 launch ekf_slam view_robot.launch.py
```

RViz displays:
- **Grid**: Reference grid in odom frame
- **TF**: Coordinate frame tree visualization
- **LaserScan**: Laser points (red)
- **Map**: SLAM-generated occupancy grid
- **RobotModel**: 3D robot visualization
- **Global/Local Plan**: Navigation paths (if Nav2 running)
- **Costmap Footprints**: Robot footprint in costmaps

### 4. Control the Robot

Use keyboard teleoperation to move the robot for mapping:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controls:**
- `i` - Forward
- `k` - Stop
- `j` - Turn left
- `l` - Turn right
- `u/o/m/,` - Diagonal movements
- `q/z` - Increase/decrease speed

### 5. Optional: Run Nav2 Navigation (if available)

```bash
ros2 launch nav2_bringup navigation_launch.py
```

Set navigation goals in RViz using "2D Goal Pose" tool.

## Quick Start (All-in-One)

Run all commands in separate terminals:

```bash
# Terminal 1: Simulation
ros2 launch ekf_slam robot_bookstore.launch.py

# Terminal 2: EKF SLAM
ros2 launch ekf_slam ekf_slam.launch.py

# Terminal 3: Visualization
ros2 launch ekf_slam view_robot.launch.py

# Terminal 4: Robot Control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Configuration

### EKF SLAM Parameters

Edit `launch/ekf_slam.launch.py` to adjust:

```python
parameters=[{
    'use_sim_time': True,
    'map_resolution': 0.05,      # Map resolution (meters/cell)
    'map_width': 400,            # Map width (cells)
    'map_height': 400,           # Map height (cells)
    'max_laser_range': 3.5,      # Maximum valid laser range (m)
    'min_laser_range': 0.12,     # Minimum valid laser range (m)
}]
```

### RViz Configuration

Modify `rviz/robot_view.rviz` or save your custom layout from RViz.

## Topics

### Subscribed Topics
- `/scan` (sensor_msgs/LaserScan) - Laser scan data
- `/odom` (nav_msgs/Odometry) - Robot odometry

### Published Topics
- `/map` (nav_msgs/OccupancyGrid) - SLAM-generated map
- `/ekf_pose` (geometry_msgs/PoseStamped) - Estimated robot pose
- `/landmarks` (geometry_msgs/PoseArray) - Detected landmarks

### TF Frames
- `map` → `odom` (published by EKF SLAM)
- `odom` → `base_footprint` (published by Gazebo diff_drive)
- `base_footprint` → `base_link` → `base_scan/imu_link/camera_*` (static TF publishers)

## Troubleshooting

### Robot falls through ground
- Check `GAZEBO_MODEL_PATH` is set correctly
- Verify bookstore world models are installed

### No laser scan in RViz
- Ensure TF tree is complete: `ros2 run tf2_tools view_frames`
- Check `/scan` topic: `ros2 topic echo /scan`
- Verify `use_sim_time: True` in all nodes

### Map not updating
- Confirm robot is moving (teleop working)
- Check EKF SLAM node is running: `ros2 node list`
- Verify `/odom` topic is publishing: `ros2 topic hz /odom`

### TF timestamp errors (controller_server)
- These come from Nav2 nodes if `use_sim_time` not set
- Won't affect SLAM functionality
- Fix by adding `use_sim_time: True` to Nav2 launch file

## Development

### File Locations
- Node implementation: `ekf_slam/ekf_slam_node.py`
- Launch files: `launch/*.launch.py`
- World files: `worlds/bookstore/bookstore.world`
- Robot models: `worlds/bookstore/models/turtlebot3_waffle_pi/`

### Building After Changes

```bash
cd ~/ekf_slam_ws
colcon build --packages-select ekf_slam --symlink-install
source install/setup.bash
```

Use `--symlink-install` to avoid rebuilding after Python file changes.

## License

Apache License 2.0

## Maintainer

- **Author**: White8848
- **Repository**: [EKF_SLAM_ROS2](https://github.com/White8848/EKF_SLAM_ROS2)
- **Email**: qinghuaharry1204@gmail.com

## Acknowledgments

- TurtleBot3 models and Gazebo worlds
- ROS 2 community
- AWS Robomaker retail models
