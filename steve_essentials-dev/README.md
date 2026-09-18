# steve_essentials

**Steve Essentials** is the core infrastructure package for the Steve Butler robot. It serves as a central repository for all hardware drivers, common message definitions, kinematics, and utility nodes required to operate the robot.

This package consolidates dependencies to ensure a streamlined setup for both simulation and real hardware deployment.

## Overview

Unlike other packages that focus on specific capabilities (navigation, perception, manipulation), `steve_essentials` provides the **foundational drivers and interfaces** that all other packages rely on. It includes:
- **Base Drivers**: Control for the Neobotix MMO-700 omnidirectional base.
- **Sensor Drivers**: Lidar and Camera drivers.
- **Input Devices**: Joystick and teleoperation nodes.
- **Common Interfaces**: Shared custom message and service definitions.

## Included Packages

This metapackage contains the following sub-packages:

### Core Neobotix Packages
- **`neo_common2`**: Common headers and configurations used across Neobotix packages.
- **`neo_msgs2`**: Custom message definitions for Neobotix robots (e.g., specific drive commands, sensor status).
- **`neo_srvs2`**: Custom service definitions (e.g., unlocking brakes, resetting drivers).
- **`neo_kinematics_omnidrive2`**: Kinematics solver for the omnidirectional mecanum wheel base.
- **`neo_relayboard_v2-2`**: Driver for the specific relayboard used in the MMO-700 to communicate with motors and power systems.
- **`neo_sick_s300-2`**: Driver for the SICK S300 safety laser scanners.
- **`neo_teleop2`**: Teleoperation nodes specifically tuned for the MMO-700 kinematics.

### Third-Party Drivers
- **`joystick_drivers`**: Standard ROS 2 joystick drivers (`joy` package) to interface with gamepads (e.g., Logitech F710).
- **`realsense-ros`**: Official ROS 2 wrapper for Intel RealSense cameras (L515, D435, etc.).

---

## Intel RealSense Setup (`realsense-ros`)

The `realsense-ros` package included here provides the driver for the **L515 LiDAR Depth Camera** mounted on the robot's pan-tilt unit.

### Key Launch Commands
To launch the camera driver independently (for testing):

```bash
ros2 launch realsense2_camera rs_launch.py \
    device_type:=l515 \
    enable_color:=true \
    enable_depth:=true \
    pointcloud.enable:=true
```

### Configuration
- **Frame ID**: The camera is typically published in the `camera_link` frame.
- **Resolution**: Configurable in the launch files, defaults to 640x480 for better performance on the robot's onboard computer.
- **Visualizing**: You can view the pointcloud and RGB stream in RViz by adding the corresponding topics (e.g., `/camera/depth/color/points`, `/camera/color/image_raw`).

---

## Contributors & Acknowledgments

This project is developed at the **Humanoid Robots Lab (HRL)**, University of Bonn.

- **Shrikar Nakhye** - [GitHub](https://github.com/Itsshriks) | [Email](mailto:nakhyeshrikar@icloud.com)
- **Pratik Adhikari** - [GitHub](https://github.com/pratik-adhikari) | [Email](mailto:pratikadhikari.de@gmail.com)
- **Riddhesh More** - [GitHub](https://github.com/RiddheshMore) | [Email](mailto:riddheshmore311@gmail.com)

### Acknowledgements
- **Rohit Menon** - For mentorship and technical guidance on Neobotix platforms.
- **Prof. Maren Bennewitz** - Head of the Humanoid Robots Lab, University of Bonn.

---

**License**: All custom code is released under the MIT License. Third-party packages retain their original licenses.