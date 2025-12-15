# TurtleBot3 Vision Line Follower (ROS 2 Humble)

Camera-driven line following for TurtleBot3 using **OpenCV** (bird’s-eye view + contour filtering) and a lightweight **/cmd_vel** controller.

> Tested on: ROS 2 Humble / Ubuntu 22.04

---

## Demo (YouTube Shorts)

<a href="https://youtube.com/shorts/grw7qH93KJ0">
  <img src="https://img.youtube.com/vi/grw7qH93KJ0/hqdefault.jpg" width="900">
</a>

---

## Overview

This package consists of two nodes:

- **`camera_viewer`**: subscribes to the camera image, builds a BEV (bird’s-eye view), extracts a dark line, and publishes a compact detection message + debug overlays.
- **`line_follower`**: consumes the detection message and generates **`geometry_msgs/Twist`** on **`/cmd_vel`**.

An optional **`rqt_image_view`** is launched to visualize the debug stream.

---

## Node Graph (High-level)

`/camera/image_raw` → **camera_viewer** → `/camera/line_detection` → **line_follower** → `/cmd_vel`  
                                     └→ `/line_follower/debug_image` (view with rqt)

---

## Topics

### Subscribed
- `/camera/image_raw` (`sensor_msgs/Image`)

### Published
- `/camera/line_detection` (`geometry_msgs/Vector3`)
  - `x`: horizontal error (line center offset from image center)
  - `y`: line coverage ratio (how much of the bottom ROI is “line”)
  - `z`: validity (1.0 = detected, 0.0 = not detected)
- `/line_follower/debug_image` (`sensor_msgs/Image`)
- `/cmd_vel` (`geometry_msgs/Twist`)
- (extra) `/image_processed`, `/camera/image_bev`

---

## Quick Start

### 1) Build
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <YOUR_REPO_URL>
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
