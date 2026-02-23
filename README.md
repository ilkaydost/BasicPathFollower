# BasicPathFollower

A minimal ROS1 Python project that drives a **tricycle robot** through a hand-drawn path in Gazebo simulation using a simple proportional controller.

## Overview

This project demonstrates path following with a tricycle robot. The user defines waypoints interactively, the path is interpolated using NumPy, and a proportional controller adjusts the robot's steering angle and velocity to track the path in real-time.

**Result videos:**
- [Path Following Demo](https://www.youtube.com/watch?v=FR37Owa-Jxk)
- [Inspiration](https://www.youtube.com/watch?v=Qh15Nol5htM)

---

## Requirements

### System
- **OS:** Ubuntu 16.04 or later
- **Python:** 3.x

### ROS
- ROS1 (Melodic/Noetic recommended)
- `tf` (transformation library)
- `nav_msgs` (navigation messages)
- Gazebo with tricycle robot model

### Python Packages
- `numpy` – path interpolation
- `matplotlib` – path visualization
- `rospy` – ROS Python client

---

## Project Structure

```
.
├── README.md           # This file
├── teleop.py           # Main ROS node (the only source file)
└── .github/
    └── copilot-instructions.md  # Domain-specific conventions
```

---s

## Installation & Setup

1. **Install ROS1** (Ubuntu 16.04+):
   ```bash
   # Follow official ROS installation guide
   sudo apt-get install ros-<distro>-desktop-full
   source /opt/ros/<distro>/setup.bash
   ```

2. **Install dependencies:**
   ```bash
   sudo apt-get install python3-numpy python3-matplotlib
   pip install numpy matplotlib rospy
   ```

3. **Set up your catkin workspace** with the tricycle robot model and source it:
   ```bash
   source /path/to/catkin_ws/devel/setup.bash
   ```

---

## Usage

### 1. Start ROS Core & Gazebo

Open two terminals:

**Terminal 1 – ROS Core:**
```bash
roscore
```

**Terminal 2 – Launch Gazebo with the tricycle:**
```bash
# Assumes your workspace contains the tricycle robot description
roslaunch <your_package> tricycle_world.launch  # (or equivalent)
```

### 2. Run the Path Follower Node

**Terminal 3:**
```bash
cd /path/to/BasicPathFollower
source /path/to/catkin_ws/devel/setup.bash
python teleop.py
```

### 3. Interactive Input

The script will prompt you for:
- **Number of waypoints:** Enter how many coordinate points to define
- **Waypoint coordinates:** For each point, enter `x` and `y` values
- **Distance tolerance:** A threshold (e.g., `0.01`) for reaching each waypoint

The path is then visualized with matplotlib, and the robot begins following it.

---

## How It Works

1. **Odometry Subscription** – The node subscribes to `/tricycle/odom` and tracks the robot's position and orientation.
2. **Path Interpolation** – NumPy interpolates user-provided waypoints into a smooth path.
3. **Proportional Control** – For each interpolated waypoint, the controller calculates:
   - **Steering angle** – target direction relative to the robot
   - **Linear velocity** – speed to the waypoint
   - **Angular velocity** – steering command
4. **Command Publishing** – Velocity commands are published on `/tricycle/cmd_vel`.
5. **Iteration** – Once within tolerance of a waypoint, the robot moves to the next one.
6. **Completion** – When all waypoints are reached, the robot stops.

---

## Code Architecture

- **Single Node:** All behavior is in the `Teleoperation` class in `teleop.py`
- **No Custom Package:** The script is standalone and relies on standard ROS messages
- **Minimal Dependencies:** Only `numpy`, `matplotlib`, and ROS core packages

### Key Methods
- `path()` – Collects user input and interpolates waypoints
- `euclidean_distance()` – Calculates distance to current waypoint
- `steering_angle()` – Calculates desired heading
- `get_steering_command()` – Determines steering/velocity based on angle difference
- `publish_velocity()` – Publishes Twist commands to the robot
- `move2goal()` – Main loop that drives the robot through all waypoints

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| **"Cannot connect to ROS master"** | Ensure `roscore` is running in a separate terminal |
| **Robot doesn't move** | Verify Gazebo is running and the tricycle model is spawned |
| **Wrong path** | Check that waypoints are entered in increasing x-order for interpolation |
| **Robot oscillates** | Increase the distance tolerance or reduce proportional gain constants |
| **Plot doesn't show** | Ensure matplotlib backend is available; add `plt.ion()` if needed |
| **Odometry is zero** | Confirm `/tricycle/odom` topic is being published; use `rostopic list` to verify |

---

## ROS Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/tricycle/odom` | `nav_msgs/Odometry` | Subscribe (robot position/orientation) |
| `/tricycle/cmd_vel` | `geometry_msgs/Twist` | Publish (velocity commands) |

---

## Notes

- **No tests:** This is a minimal project with no automated test suite.
- **Hardcoded limits:** The path is limited to 9 interpolated points by default (configurable).
- **Single-threaded:** Matplotlib blocking calls synchronously during path visualization.
- **ROS1 only:** Not compatible with ROS2 without significant changes.

---

## Future Improvements

- Dynamic path length (not limited to 9 points)
- Better steering controller (PID instead of proportional)
- Obstacle avoidance integration
- ROS2 compatibility

---

## Author & License

**Author:** Ilkay Dost  
**Created:** January 2021  
**Updated:** January 2026  
**Status:** Educational/Demonstration project


