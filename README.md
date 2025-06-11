# Turtlebot Square at Origin

This ROS node moves a TurtleBot3 robot to the world origin (0,0,0) and then commands it to drive in a 2 m × 2 m square starting from the origin.

---

## Table of Contents

* [Features](#features)
* [Prerequisites](#prerequisites)
* [Installation](#installation)
* [Usage](#usage)
* [Node Details](#node-details)

  * [Subscriptions](#subscriptions)
  * [Publications](#publications)
* [Parameters & Constants](#parameters--constants)
* [Launch Example](#launch-example)
* [Troubleshooting](#troubleshooting)

---

## Features

* **Automatic homing**: Rotates and drives the robot to the origin until within 5 cm.
* **Orientation alignment**: Once at the origin, the robot aligns its yaw to 0 rad.
* **Square trajectory**: Drives a 2 m × 2 m square path with yaw correction and precise 90° rotations.

## Prerequisites

* ROS (tested on Melodic/Noetic)
* TurtleBot3 packages installed:

  * `turtlebot3_description`
  * `turtlebot3_teleop`
  * `turtlebot3_bringup`
* A working TurtleBot3 simulation or hardware setup
* Python dependencies: `rospy`, `geometry_msgs`, `nav_msgs`, `tf`

## Installation

1. Clone or copy this script into your ROS workspace, for example:

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/rseteam4/turtlebot3_square.git
   ```
2. Make sure the script is executable:

   ```bash
   chmod +x turtlebot3_square_at_origin/scripts/turtlebot3_square_at_origin.py
   ```
3. Build your workspace:

   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

## Usage

Run the node directly or via a launch file.

```bash
rosrun turtlebot3_square_at_origin turtlebot3_square_at_origin.py
```

Alternatively, include it in a launch:

```xml
<launch>
  <!-- Set TurtleBot3 model if needed -->
  <env name="TURTLEBOT3_MODEL" value="burger" />

  <!-- Bring up core and robot state publisher -->
  <include file="$(find turtlebot3_bringup)/launch/turtlebot3_robot.launch" />

  <!-- Start square movement node -->
  <node pkg="turtlebot3_square_at_origin"
        type="turtlebot3_square_at_origin.py"
        name="turtlebot3_square_at_origin"
        output="screen" />
</launch>
```

## Node Details

### Subscriptions

* `/odom` (`nav_msgs/Odometry`): Robot odometry for position and yaw feedback.

### Publications

* `/cmd_vel` (`geometry_msgs/Twist`): Velocity commands for driving and rotating.

## Parameters & Constants

| Name               | Purpose                                                   | Default |
| ------------------ | --------------------------------------------------------- | ------- |
| `tol_pos`          | Position tolerance when homing (m)                        | `0.05`  |
| `tol_yaw`          | Yaw tolerance for rotations (rad)                         | `0.02`  |
| `forward_distance` | Length of each square side (m)                            | `2.0`   |
| `forward_speed`    | Linear speed for forward motion (m/s)                     | `0.2`   |
| `angular_speed`    | Angular speed for rotations (rad/s)                       | `0.5`   |
| `k_correction`     | Proportional gain for yaw drift correction during driving | `1.0`   |

## Launch Example

```bash
roslaunch turtlebot3_square_at_origin square_at_origin.launch
```

Include a `square_at_origin.launch` file in your package to wrap the above `<launch>` snippet.

## Troubleshooting

* **No odometry data**: Ensure the robot or simulation is publishing `/odom`.
* **Robot drifts off path**: Adjust `k_correction` to improve yaw correction.
* **Robot fails to rotate accurately**: Verify odom-based yaw is correct and the IMU is calibrated.
* **Permission denied**: Make script executable and rebuild your workspace.

---


