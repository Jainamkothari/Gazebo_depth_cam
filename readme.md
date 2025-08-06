# Simulated Depth Camera Setup for ROS 2 + Gazebo Classic

This package provides a lightweight, modular setup to simulate a depth camera in **ROS 2** using **Gazebo Classic**. It can be easily integrated into larger robotics projects that require simulated perception.

---

## 📦 Overview

- ✅ Simulates a depth camera using a Gazebo Classic plugin
- ✅ Publishes depth and RGB topics via `image_transport`
- ✅ Includes example URDF and launch files
- 🧩 Designed to be **modular and reusable** in other projects

---

## 🗂 Project Structure

depth_ws/
└── src/
└── simple_depth_cam/
├── launch/
│ ├── gazebo_classic.launch.py
├── urdf/
│ ├── depth_cam.urdf.xacro
└── ...


---

## 🚀 How to Use

### 1. Build the workspace
```bash
cd depth_ws
colcon build --symlink-install
source install/setup.bash

2. Launch the Gazebo simulation

ros2 launch simple_depth_cam gazebo_classic.launch.py

📡 Topics Published

Once launched, the camera publishes:

    /camera/depth/image_raw — Depth image

    /camera/color/image_raw — RGB image

    /camera/camera_info — Camera calibration info

    /tf — Frame transforms (optional; to be added)

⚙️ Configurable Parameters

The xacro file allows customization:

    Camera resolution

    FOV

    Position/orientation on robot

🔧 Integration Notes

    This package does not include MoveIt, navigation, or SLAM logic.

    It is designed to drop into other projects as a simulated perception module.

    You can easily remap the camera topics or add TF broadcasting.

🚧 Known Gaps / TODOs

RViz config file for visual debugging

Proper TF broadcasting from camera frame

Depth image filtering example

    Dynamic parameter tuning

🔄 Reusability

To reuse this in your own robot project:

    Copy urdf/depth_cam.urdf.xacro into your robot URDF.

    Include the depth cam launch in your own launch file.

    Remap topics or TF as needed.