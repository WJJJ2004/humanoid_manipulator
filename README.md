# humanoid_manipulator
Intelligent Robot Competition – Humanoid Robot Sports

## Outline
> ROS2 package for autonomous control of a humanoid manipulator.  
> Supports Pinocchio-based IK computation, motion generation, and Teleop/Auto modes.

---

## 📁 Table of Contents

1. [Overview](#overview)  
2. [Installation & Execution](#installation--execution)  
3. [Parameter Tuning](#parameter-tuning)  
4. [Development Environment](#development-environment)  
5. [Dependencies](#dependencies)  

---

## Overview
The **humanoid_manipulator** package provides inverse kinematics, motion control, and collision-aware manipulation for a humanoid robot.  
It integrates:
- In the ROS2 environment, end-effector trajectories are generated according to the assigned task. Inverse kinematics (IK) is then computed to generate corresponding arm motions.
- The Pinocchio library is used within ROS2 to perform collision checking based on the geometric models defined in the URDF. This ensures the safety of arm motions before execution.
- The generated arm motions are blended with lower-body motions created using the Motion Teacher, allowing the whole robot to move in a stable manner. Through this process, the arm and lower body are controlled synchronously.

### Stop movement for arm movements corresponding to self-collision
https://github.com/user-attachments/assets/2fbb4578-f151-4b23-b5c8-d9b63be6aeed

### Generate ball-catching arm movements based on the robot's viewpoint ball position
[스크린캐스트 09-29-2025 06_30_50 PM.webm](https://github.com/user-attachments/assets/cf3b055e-b6f5-40f3-8d1e-f2a122e52c54)

### A scene where the ball is caught using actual hardware
![humanoid_manipulator (1)](https://github.com/user-attachments/assets/987cc9b8-03bd-45f1-be90-3169db007b26)

---

## Installation & Execution

### Run Teleop Mode
```bash

ros2 launch humanoid_manipulator teleop_mode.launch.py

```
### Run RVIZ2 debug Mode
```bash

ros2 launch humanoid_manipulator debug_mode.launch.py

```
### Run Auto Control Mode
```bash

ros2 launch humanoid_manipulator auto_mode.launch.py

```
### Run EE Teleop Node (keyboard input)
```bash

ros2 run humanoid_manipulator ee_teleop_node

```

## Parameter Tuning

Parameters are provided in config/ros_param.yaml.
Key options:

- MAX_ITERATIONS, POSITION_TOLERANCE, SE3_TOLERANCE

- USE_REFERENCE (SRDF reference configuration)

- IK_TILT_ONLY (z-axis alignment only mode)

- Orientation weights: YAW_WEIGHT, ROLL_WEIGHT, PITCH_WEIGHT

## Development Environment
| Component   | Version                     |
|-------------|-----------------------------|
| OS          | Ubuntu 22.04, Python 3.10.12|
| Language    | C++17                       |
| ROS2        | Humble                      |
| Build Tool  | CMake 3.16+                 |

## Dependencies
- ROS2 Humble core packages

- Pinocchio

- hpp-fcl (coal)

- Eigen3

- yaml-cpp
