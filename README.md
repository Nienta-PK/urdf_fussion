# URDF_Test: Robotics Control Mini-Project (On Docker)

A ROS Noetic mini-project focused on **precision control of a robotic arm simulated in Gazebo**, using a **URDF model exported from Autodesk Fusion 360**.
This repository serves as a practical, end-to-end reference for:

* Importing CAD → URDF correctly
* Handling gravity, inertia, and joint stability in Gazebo
* Wiring ROS controllers with namespaces and `robot_description`
* Running ROS Noetic reliably inside Docker

---

## 📌 Project Goals

* Validate URDF exported from Fusion 360
* Ensure a correct TF tree and joint definitions
* Achieve stable physics simulation for heavy links
* Configure and spawn ROS controllers correctly
* Provide a clean, repeatable Docker-based workflow

---

## 🛠 Prerequisites

### 1. System Requirements

* Docker
* X11 forwarding enabled (host → container)
* Inside Docker: **Ubuntu 20.04**

> 💡 Tip: Make sure your host allows X11 access for Docker (`xhost +local:docker`).

---

### 2. ROS

* ROS Noetic (running inside Docker)
* `catkin_tools`

---

### 3. Fusion 360 → URDF Export Plugin

| ROS Version    | Plugin                                                                                         |
| -------------- | ---------------------------------------------------------------------------------------------- |
| ROS 1          | [https://github.com/syuntoku14/fusion2urdf](https://github.com/syuntoku14/fusion2urdf)         |
| ROS 2 (future) | [https://github.com/dheena2k2/fusion2urdf-ros2](https://github.com/dheena2k2/fusion2urdf-ros2) |

**Installation Path (Windows):**

```
%AppData%\Roaming\Autodesk\Autodesk Fusion 360\API\Scripts
```

---

## 🐳 Docker Environment

This project assumes ROS Noetic runs inside a Docker container named `noetic_dev`.

Add the following aliases to your `~/.bashrc` or `~/.zshrc` for convenience:

```bash
alias ros-noetic='xhost +local:docker > /dev/null && docker start noetic_dev && docker exec -it noetic_dev bash'
alias ros-join='docker exec -it noetic_dev bash'
```

Usage:

```bash
ros-noetic   # start container and attach
ros-join     # attach to an already running container
```

---

## 🏗 Workspace Installation

### 1. Create Catkin Workspace

```bash
mkdir -p ~/catkin_ws/src
```

### 2. Move Package

```bash
mv ~/urdf_test_description ~/catkin_ws/src/
```

### 3. Normalize Package Naming

Fusion export tools often generate inconsistent names. Standardize everything:

```bash
cd ~/catkin_ws/src/urdf_test_description
find . -type f -exec sed -i 's/URDF_Test_description/urdf_test_description/g' {} +
```

### 4. Build Workspace

```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

---

## 🚀 Running the Simulation

### 1. URDF Visualization (RViz)

Use this step to verify:

* TF tree correctness
* Joint hierarchy
* Joint axis orientation

```bash
roslaunch urdf_test_description display.launch
```

Expected topics:

* `/tf`
* `/tf_static`
* `/joint_states`

---

### 2. Physics Simulation (Gazebo)

```bash
roslaunch urdf_test_description gazebo.launch
```

This launches:

* Gazebo world
* URDF robot model
* `gazebo_ros_control` plugin
* ROS controllers

---

## ⚙ Controller Architecture

### Key Requirements

Controllers **WILL NOT WORK** unless **all** of the following are correct:

* `robot_description` is loaded on the parameter server
* Namespace usage is consistent everywhere
* Controller YAML joint names match the URDF **exactly**

### Typical Flow

1. Load URDF → parameter server
2. Load controller YAML → parameter server
3. Spawn controllers via `controller_spawner`

---

## 🧠 Common Pitfalls & Checks

### ✅ TF Prefix

When using multiple robots:

* Apply `tf_prefix` or namespaces consistently
* Verify TF tree with `rqt_tf_tree` or RViz

---

### ✅ Namespace Consistency

Ensure these all match:

* `robotNamespace` in the Gazebo plugin
* Controller spawner namespace
* Controller YAML namespace

---

### ✅ robot_description

Controllers **require** this parameter:

```bash
rosparam get /<your_robot_name>_robot_description
```

or

```bash
rosparam get /robot_description
```

If missing → controllers may fail **silently**.

---

### ✅ Joint Stability (Heavy Links)

If the robot collapses or oscillates:

* Increase joint damping
* Increase effort limits
* Verify inertia values from CAD
* Ensure gravity compensation via controllers
* Try increasing **P gain** gradually

---

## 🔍 Debugging Tips

Useful commands:

```bash
rosnode list
rosparam list
gz topic -l
rosservice list | grep controller
```

Check controller output:

```bash
rostopic echo /urdf_1/joint_states
```

---

## 📬 Notes

If something behaves strangely, **assume naming or namespace mismatch first**.
From experience, ~90% of issues in this project come from:

* Wrong joint name
* Wrong namespace
* Missing `robot_description`
