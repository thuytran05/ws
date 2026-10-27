# linorobot2 Overview

## Introduction

Welcome to the linorobot2 documentation. This set of guides walks you through everything you need to go from a bare robot with wheels and motors to a fully autonomous robot that can build maps of its environment and navigate them on its own. It includes how to build
a "digital twin" of your Physical Robot and run it in simulation.

These docs are structured as a tutorial sequence, with each section building on the previous one, explaining not just *how* to configure something but *why* it exists and what role it plays in the larger system. The approach is inspired by the [Nav2 setup guides](https://docs.nav2.org/setup_guides/index.html) and the original [linorobot](https://github.com/linorobot/linorobot) ROS1 wiki.

linorobot2 supports three robot base configurations
(differential drive, skid steer, mecanum drive) and integrates with ROS2, Nav2, SLAM Toolbox, and robot_localization out of the box.

---

## Why linorobot2

If you're planning to build your own custom ROS2 robot that's differential
drive, skid-steer, or mecanum drive using commonly-used parts and a
microcontroller for low-level control, then the linorobot2 system may
be a good fit for you. It provides these benefits:

- Easy adaptation to a variety of different Physical Robots through
highly parameterized configuration files 

- Develop and test Robot Software in a Gazebo simulation

- Predictable timing for high-frequency control loops. The Microcontroller
runs Robot Firmware that executes low-level high-frequency control loops
on a Physical Robot more predictably than a linux computer can.

- Extending ROS topic subscribers and publishers out to the
Microcontroller makes it easy to add custom robot hardware controlled by
the Microcontroller. You don't need to modify the communication channel to
support new messages. This is enabled by using micro-ros for passing ROS
messages between Robot Firmware and Robot Software. There is a standard
process for defining custom messages which makes it easy to extend to
support custom message types that are exchanged between Robot Firmware
and Robot Software.

- Easier to keep robot customizations up to date with later versions of
linorobot2 software and firmware. You don't want to get trapped in an
old version of ROS2 when they get upgraded
because it's too hard to redo your customizations.
Using the architected extension points in Robot Firmware and Robot Software
properly can ease future migration.

### Why not linorobot2

Linorobot2 is not a solution to all mobile-robot problems. If you're focus is
certain kinds of things, linorobot2 may not be for you:

- Non-ROS robots: linorobot2 is primarily a set of launch and configuration files
that work with ROS. If you don't intend your robot to use ROS, this software
is not for you.

- AI: Linorobot2 doesn't have explicit connections to AI. If you want
AI to navigate your robot, linorobot2 might not have much to offer beyond
its interfaces at the base-controller level and at the Nav2 stack level.

---

## Terminology

The following terms are used capitalized throughout these docs.
Their meaning is defined here.

- **Physical Robot**: The mobile base that exists in the real world
and is comprised of a body, wheels, motors, sensors, the Microcontroller,
and other physical hardware that moves in the real world.

- **Simulated Robot**: The mobile base that gets instantiated in a Gazebo
simulation, and moves in a simulated world and has the same subscribed and
published topics found in a Physical Robot.

- **Robot Software**: The ROS software including linorobot2 and Nav2
and other ROS packages that run on the Robot Computer and control
a Physical Robot or a Simulated Robot.

- **Robot Firmware**: The firmware that runs on the Microcontroller
on a Physical Robot and handles the motors and sensors
at the hardware interface level.

- **Workstation**: The computer used to view ROS' GUI displays, and to
build and load firmware onto the microcontroller on a Physical Robot. In
some configurations it may also act as the Robot Computer by running
Robot Software. In other configurations it may also act as
the Simulation Computer by running the Gazebo simulator and Robot
Software.

- **Robot Computer**: The computer that runs the Robot
Software. It is a conceptual computer, and may be a physically distinct
computer, such as a Raspberry Pi mounted on a Physical Robot, or may be
the Workstation running the Robot Software

- **Simulation Computer**: The computer that runs Gazebo and the ROS
simulation software. It is a conceptual computer, and may be a physically
distinct computer, such as a compute node in the cloud, or may be the
Workstation running simulation.

- **Base Controller**: The Microcontroller and the sensors and motor
controllers it's connected to, which together control Physical Robot
wheel movement and feed sensor data back to Robot Software.

- **Microcontroller**: The ESP32 or Pico or Pico2 microcontroller and
its Robot Firmware which runs the low-level motor-control, sensing and control
loops onboard the Physical Robot.

---

## System Requirements

### Physical Robot

A Physical Robot should meet these requirements:

- A single Microcontroller runs motor controllers, encoders and
(optionally) an IMU.
- The robot has a supported lidar or a depth camera
- The Microcontroller should be a Pico or Pico2 or supported version
of ESP32. The Teensy microcontroller is deprecated but should still work.
- Differential drive, skid-steer or meccanum drive type. Ackermann steered
robots are not yet supported.

Not all the requirements are "hard requirements", but if they are not met,
modifications to linorobot2 software or firmware will be required to make
it work.

### Robot Computer

The Robot Computer must meet these requirements:

- At least a Raspberry Pi 4 level of performance
- Ubuntu 24.04 plus ROS-Jazzy, or Docker support

### Workstation

- Must be able to run PlatformIO to build Robot Firmware for a Physical Robot.

---

## Architecture

The architecture of the system within which a linorobot2 robot runs
is described [here](architecture/). 
Architectural goals and the Robot Software and Firmware
architecture are also described.

---

## System Configurations

The linorobot2 software and firmware supports four system configurations,
two for a Physical Robot and two for a Simulated Robot:

### Physical Robot System Configurations

1. Onboard Robot Computer Configuration: A computer mounted on the
robot runs Robot Software which communicates with Robot Firmware
using a serial link. This is considered the "typical" configuration.
2. Robot Wifi Configuration: Robot Software runs on the Workstation
and communicates with Robot Firmware using wifi. This configuration
supports very low-cost robots but does rely on very good wifi.

### Simulated Robot System Configurations

1. Simulation Configuration: Robot Software and the Gazebo simulator
run on the Workstation.
2. Cloud Simulation Configuration: Robot Software and the Gazebo
simulator run on a compute node in the cloud. Gui tools
are displayed on a virtual monitor in the compute node. VNC runs in
a browser on the Workstation and connects to the virtual monitor
in the compute node, allowing the Workstation user to interact
with the GUI tools and launch Robot Software and the Simulated
Robot in the cloud.

The supported system configurations of linorobot2 are described,
along with block diagrams [here](/system_configurations/)

---

# Documentation Overview

## The Journey: Bare Robot → Autonomous Physical Robot with digital twin

Here's the full path from hardware to autonomous navigation, in order:

---

### 1. [Installation](installation/)

Get linorobot2 Robot Software installed on both your Robot Computer and your
Workstation. Get Robot Firmware built and installed on your Microcontroller.
Covers the one-command `install.bash` setup script, the
supported robot types and sensors, points to documentation on how to
build and install the Robot Firmware for motor control etc, and how to
install the RViz visualization package for remote debugging. Also covers
the Gazebo simulation setup and the Docker option for host machines not
running Ubuntu 24.04.

---

### 2. [Base Controller](base_controller/)

The base controller is the bridge between Robot Software and your robot's motors and sensors. The Microcontroller (e.g., Pico or ESP32) running micro-ROS handles the low-level work: receiving `/cmd_vel` velocity commands, driving the motor controllers, reading wheel encoders, and publishing raw odometry and IMU data. This section explains the hardware architecture, the published/subscribed topics, and how to bring up the microcontroller agent. It points to documentation on how to build and configure the Base Controller.

---

### 3. [Odometry](odometry/)

Odometry is the robot's estimate of its position based on how far its wheels have turned. Wheel odometry alone drifts over time, as slip, uneven terrain, and mechanical tolerances all accumulate errors. This section explains how the `robot_localization` package uses an Extended Kalman Filter (EKF) to fuse wheel odometry with IMU data, producing a much more reliable filtered odometry on `/odometry/filtered`. It points to documentation on how to configure and calibrate your encoders in Robot Firmware. It also covers how to verify your odometry before running SLAM.

---

### 4. [Setting Up Sensors](sensors/)

Sensors are what let the robot perceive its environment. linorobot2 uses 2D lidar (LaserScan) as the primary sensor for mapping and obstacle detection, with optional depth cameras (RGBD) for detecting obstacles outside the lidar's scan plane. This section covers the supported real hardware sensors and how to configure them via the install script, as well as how simulation sensors are defined in URDF/xacro files and how to adjust their positions on the robot.

---

### 5. [Setting Up Transforms (TF)](transforms/)

TF2 is the ROS2 system that tracks the spatial relationship between every coordinate frame on the robot, from the global map frame down to each individual sensor. This section explains the full frame hierarchy (`map → odom → base_footprint → base_link → sensors`), what publishes each transform, and how to configure sensor positions in the URDF properties files. Includes verification steps using `view_frames` and RViz.

---

### 6. [Mapping](mapping/)

Before autonomous navigation is possible, the robot needs a map. This section covers SLAM (Simultaneous Localization and Mapping) using SLAM Toolbox, including how it works conceptually (scan matching and loop closure), the key configuration parameters in `slam.yaml`, and the step-by-step process for driving the robot through an environment, building the map, and saving it to disk.

---

### 7. [Navigation](navigation/)

With a map in hand, Nav2 can navigate the robot autonomously. This section gives an overview of Nav2's components (AMCL, global planner, local controller, costmaps, behavior tree), walks through launching navigation with a pre-built map, and explains how to set the robot's initial pose and send navigation goals in RViz. Importantly, it covers the **robot footprint**, the one parameter you must configure to match your robot's actual size.

---

### 8. [Docker](docker/)

Covers running linorobot2 without a native ROS2 installation using Docker. Includes configuring the Docker environment for hardware, simulation, or CUDA-accelerated simulation; building the image; web-based visualization via KasmVNC (accessible in any browser at `http://<host_ip>:3000`); running the Physical Robot or Simulated Robot via Tmuxinator profiles; and creating custom profiles for your own workflow.

---

### 9. [Simulation Tools](tools/)

Documents the two Gazebo world generation tools in `linorobot2_gazebo`. `image_to_gazebo` is a GUI tool that converts any floor plan image into a Gazebo world by calibrating its scale and origin interactively. `create_worlds_from_maps` is a batch CLI tool that converts all saved SLAM maps into Gazebo worlds in one command. Both let you simulate in the exact same environment your robot operates in physically.

---

## Quick Reference: Key Files

| File | Purpose |
|------|---------|
| `install.bash` | One-command robot computer setup |
| `linorobot2_base/config/ekf.yaml` | EKF sensor fusion configuration |
| `linorobot2_description/urdf/<robot>_properties.urdf.xacro` | Robot dimensions and sensor poses |
| `linorobot2_navigation/config/slam.yaml` | SLAM Toolbox configuration |
| `linorobot2_navigation/config/navigation.yaml` | Nav2 configuration (including footprint) |
| `linorobot2_navigation/maps/` | Where to save maps |

## Quick Reference: Key Launch Commands

| Command | Purpose |
|---------|---------|
| `ros2 launch linorobot2_bringup bringup.launch.py` | Boot the Physical Robot |
| `ros2 launch linorobot2_gazebo gazebo.launch.py` | Start Simulated Robot |
| `ros2 launch linorobot2_navigation slam.launch.py` | Start SLAM (mapping) |
| `ros2 launch linorobot2_navigation navigation.launch.py map:=<map>.yaml` | Start Nav2 (navigation) |
| `ros2 launch linorobot2_viz slam.launch.py` | Visualize SLAM from host machine |
| `ros2 launch linorobot2_viz navigation.launch.py` | Visualize navigation from host machine |

## Help and Support

Issues and Discussions can be filed in the usual way in the github repos for
linorobot2 and linorobot2_hardware. In addition,
anyone is welcome to join the [linorobot google group(https://groups.google.com/g/linorobot) and ask questions or discuss matters relevant to the project.

---
## External Resources

- [Nav2 Setup Guides](https://docs.nav2.org/setup_guides/index.html)
- [Nav2 Configuration Guide](https://docs.nav2.org/configuration/index.html)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [robot_localization](https://docs.ros.org/en/ros2_packages/rolling/api/robot_localization/index.html)
- [linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware): microcontroller firmware and hardware build guide
- [Gazebo ROS2 Overview](https://gazebosim.org/docs/latest/ros2_overview)
