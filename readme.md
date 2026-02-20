# AMR Project — Autonomous Mobile Robot with ROS 2 & Ignition Gazebo Fortress

A differential-drive AMR simulation built with ROS 2 Humble and Ignition Gazebo Fortress. [Mr_robot_description](https://github.com/atom-robotics-lab/mr_robot_description) package by [atom_robotics_lab](https://github.com/atom-robotics-lab) was used and modified plugins were used for gazebo fortress. The robot navigates autonomously in a custom gazebo world using Nav2.

---

## 📦 Packages

| Package | Description |
|---|---|
| `amr_navigation` | Launch files, Nav2 params, world SDF, RViz config, BT XML |
| `mr_robot_description` | URDF/xacro, meshes, Gazebo plugins (submodule) |

---

## 🖥️ System Requirements

- **OS:** Ubuntu 22.04 (Jammy)
- **ROS 2:** Humble Hawksbill
- **Gazebo:** Ignition Fortress (ign-gazebo 6)

---

## ⚙️ Installation

### 1. Install ROS 2 Humble

visit ros2's [website](https://docs.ros.org/en/humble/Installation.html) for installation.

### 2. Install Ignition Fortress

visit ignition gazebo's [webstie](https://gazebosim.org/docs/fortress/install/) for installation.

### 3. Install Nav2, ROS-Ignition Bridge, other required packages

```bash
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-slam-toolbox \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro
```

### 4. Cloning repository in your ros2 workspace

```bash
git clone --recurse-submodules https://github.com/Kartik9250/amr_project.git
```

> **Important:** The `--recurse-submodules` flag is required to also pull `mr_robot_description`.
> If you already cloned without it, run:
> ```bash
> git submodule update --init --recursive
> ```

---

## 🚀 Running the Simulation

### Launch Everything

```bash
ros2 launch amr world.launch.py
```

This starts:
- Ignition Gazebo Fortress with the custom obstacle world
- `robot_state_publisher` with the AMR URDF
- ROS–Gazebo bridge (cmd_vel, odom, scan, imu, clock, tf, joint_states)
- Nav2 stack (AMCL, planner, controller, behavior server, bt_navigator)
- RViz2 with full navigation visualisation

---

## 🗺️ Mapping (SLAM)

If you want to build a map before navigating:

```bash
ros2 launch amr world.launch.py
# In a second terminal:
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```
> **Important:** comment out the `nav2` object in `world_launch.py` before running.

Drive the robot manually to build the map:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

Save the map when complete:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

---

## 🧭 Navigation

Once the simulation is running:

1. Open RViz (launched automatically)
2. Use **2D Pose Estimate** tool to set the robot's initial position on the map (if required)
3. Use **2D Goal Pose** tool to send a navigation goal
4. The robot will plan a path and navigate autonomously, recovering if blocked

### What you can see in RViz

| Display | Topic | Colour | What it shows |
|---|---|---|---|
| Static Map | `/map` | Grey | Pre-built occupancy map |
| Global Costmap | `/global_costmap/costmap` | Costmap scheme | Inflation around known obstacles |
| Local Costmap | `/local_costmap/costmap` | Costmap scheme | Live obstacle window around robot |
| Global Path | `/received_global_plan` | Blue | Full planned route to goal |
| Local Path | `/local_plan` | Green | What controller is currently following |
| Planner Path | `/plan` | Orange | Raw path before smoothing |
| Laser Scan | `/scan` | Red | Live LiDAR point cloud |
| AMCL Particles | `/particle_cloud` | Cyan | Localisation confidence (tighter = better) |
| Robot Footprint | `/local_costmap/published_footprint` | Green outline | Collision footprint used by planner |

---

## 🤖 Robot Specifications

| Parameter | Value |
|---|---|
| Drive type | Differential drive |
| Wheel separation | 199 mm |
| Wheel radius | 53.5 mm |
| Max linear velocity | 0.5 m/s |
| Max angular velocity | 1.0 rad/s |
| Sensor | 2D LiDAR (360°, 4.5 m range) |
| Arena size | 20 × 20 m |

---

## 🛠️ Troubleshooting

### Gazebo crashes on camera rotation
Switch renderer from `ogre2` to `ogre` in `world.sdf`:
```xml
<engine>ogre</engine>
```
Or set via environment:
```bash
export IGN_GAZEBO_RENDER_ENGINE=ogre
```

### Shared library plugin errors
```
Failed to load system plugin [libignition-sim-scene-system]
```
Set plugin paths:
```bash
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins
export IGN_GUI_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/ign-gui-6/plugins
```

### Wheel joints missing from TF tree
Add `<topic>` to the joint state publisher plugin in `mr_robot_plugins.gazebo`:
```xml
<plugin filename="libignition-gazebo-joint-state-publisher-system.so"
        name="ignition::gazebo::systems::JointStatePublisher">
  <topic>/joint_states</topic>
</plugin>
```

### Robot gets stuck and aborts navigation goal
This is a Nav2 recovery issue. Ensure `failure_tolerance` in `nav2_params.yaml` is set to at least `10.0` and the BT XML includes a `RecoveryNode` wrapping the main navigation sequence.

### bt_navigator fails: `ID already registered`
You have a duplicate entry in `plugin_lib_names` in `nav2_params.yaml`. Search for and remove the duplicate `nav2_goal_updated_condition_bt_node` entry.

---

## 📁 Project Structure

```
amr_project/
├── amr_navigation/
│   ├── config/
│   │   ├── nav2_params.yaml        # Nav2 full parameter config
│   │   └── navigate_to_pose.xml    # Behaviour Tree XML
│   ├── config.rviz               # RViz configuration
│   ├── worlds/
│   │   └── world.sdf               # 20x20m obstacle arena
│   ├── launch/
│   │   └── world.launch.py         # Main launch file
│   └── maps/                       # Saved maps go here
├── mr_robot_description/           # Git submodule
│   ├── urdf/
│   │   ├── materials.xacro
│   │   ├── mr_robot.trans
│   │   └── mr_robot.xacro
│   ├── gazebo/
│   │   ├── mr_robot.gazebo
│   │   └── mr_robot_plugins.gazebo
│   └── meshes/
└── .gitmodules
```