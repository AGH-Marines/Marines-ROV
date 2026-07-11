# Project Specification: Autonomous 3D Navigation and Mapping System for an Underwater Robot (ROV)

## 1. Project Goal

The main goal of the project is to build a modular, reliable, and scalable control and perception system for an underwater robot (ROV) in the ROS2 (Humble) environment. The robot's task is to carry out autonomous missions consisting of safely moving from point A to point B along the shortest (optimal) path, while effectively avoiding static and dynamic obstacles in three-dimensional space.

**Key functional assumptions:**

- **Station Keeping:** The robot actively compensates for external forces (buoyancy, water currents) and maintains a set depth ($Z$) and orientation ($Yaw$) using cascaded PID controllers.
- **3D Perception and Mapping:** The system processes raw data from a multibeam sonar or stereo cameras and builds a continuous occupancy map of space in the form of an octree (OctoMap).
- **Autonomous Path Planning:** The system determines a safe, collision-free trajectory in 3D space (6 DoF).
- **Hardware Abstraction (Portability):** The architecture separates the simulation layer from the robot's business logic. Porting the system to a physical robot only requires swapping the input package (sensor drivers) and the output package (motor mixer).
- **TF2-based Architecture:** All localization and control rely on queries to the dynamic transform tree (tf2), eliminating rigid dependencies on odometry topics.

## 2. Operating Modes (Top-Level Launchers)

The control system provides two main operating modes, launched from the integration package:

**Mode 1: Gamepad Control (Manual/Semi-Autonomous)**

- Launcher: `teleop_joy.launch.py`
- Description: The operator controls the robot's translational motion (axes $X, Y$) using the controller sticks (e.g. Xbox/PlayStation). The vertical axis ($Z$) and yaw ($Yaw$) are optionally controlled by an automatic stabilization node. When the operator releases the sticks, the robot automatically "locks" its position in the water.
- Use case: Inspection missions, preliminary tank mapping, emergency procedures.

**Mode 2: Full Autonomy (Point A -> B Transit)**

- Launcher: `autonomy.launch.py`
- Description: The operator (or an external state machine) provides target coordinates ($X, Y, Z, Yaw$). The navigation package analyzes the current OctoMap structure, generates the optimal route, and passes it to the trajectory controller. The robot autonomously completes the entire transit, continuously correcting its route whenever new obstacles are detected.

## 3. Workspace Folder Structure

```
ros2_ws/
├── Dockerfile
├── README.md
└── src/
    ├── rov_bringup/
    │   ├── launch/
    │   │   ├── autonomy.launch.py
    │   │   └── teleop_joy.launch.py
    │   └── rviz/
    │       └── default_view.rviz
    ├── rov_simulation/
    │   ├── data/              # 3D models (.obj, .dae) of the robot and obstacles
    │   ├── launch/
    │   │   └── sim_launch.py
    │   └── scenarios/
    │       └── underwater_world.scn
    ├── rov_localization/
    │   ├── config/
    │   │   └── ekf.yaml
    │   └── launch/
    │       └── localization_launch.py
    ├── rov_mapping/
    │   └── launch/
    │       └── mapping_launch.py
    ├── rov_navigation/
    │   ├── launch/
    │   │   └── navigation_launch.py
    │   └── rov_navigation/
    │       ├── __init__.py
    │       └── planner_node.py
    ├── rov_control/
    │   ├── launch/
    │   │   └── control_launch.py
    │   └── rov_control/
    │       ├── __init__.py
    │       └── pid_controller_node.py
    ├── rov_actuation/
    │   ├── launch/
    │   │   └── actuation_launch.py
    │   └── rov_actuation/
    │       ├── __init__.py
    │       └── thruster_allocator_node.py
    └── rov_mission/
        ├── launch/
        │   └── mission_launch.py
        └── rov_mission/
            ├── __init__.py
            └── state_machine_node.py
```

## 4. Detailed Package Description

**1. rov_simulation (Environment Layer)**
- Type: `ament_cmake` (manages assets and scene files for Stonefish)
- Task: Handles world physics, robot hydrodynamics, and models gravity and buoyancy. Generates artificial noise for the sensors.
- Inputs: `/thrusters/STONEFISH` (`std_msgs/Float64MultiArray` - thruster thrust vector)
- Outputs: Raw sensor topics: `/marines_rov/odometry`, `/marines_rov/imu`, `/marines_rov/dvl`, `/marines_rov/point_cloud`.

**2. rov_localization (State Estimation Layer)**
- Type: Integration of external ROS2 tools (`robot_localization`)
- Task: Fuses data from the IMU (orientation, accelerations), DVL (linear velocity relative to the seabed), and kinematic odometry using an Extended Kalman Filter (EKF).
- Key function: Filters out noise and publishes the dynamic relation in the transform tree: `odom -> base_link`.
- Outputs: `/odometry/filtered` (`nav_msgs/Odometry`), TF2 transform.

**3. rov_mapping (3D Perception Layer)**
- Type: Integration of external ROS2 tools (`octomap_server`)
- Task: Builds a three-dimensional map of the environment. Consumes the point cloud from the virtual sonar, obtains the robot's current position via tf2 (`odom -> base_link` relation), and projects the points into a spatial voxel database.
- Outputs: `/octomap_binary`, `/octomap_full` (`octomap_msgs`), `/occupied_cells_vis_array` (visualization of 3D structures in RViz2).

**4. rov_navigation (Route Planning Layer)**
- Type: `ament_python`
- Task: Implements a shortest-safe-path search algorithm (e.g. RRT* or 3D A*). Reacts to commands to move to a target point. Tracks space occupancy based on the OctoMap.
- Inputs: Occupancy map, `/marines_rov/goal_pose` (`geometry_msgs/PoseStamped`)
- Outputs: `/marines_rov/global_path` (`nav_msgs/Path` - list of route waypoints).

**5. rov_control (Control and Trajectory Tracking Layer)**
- Type: `ament_python`
- Task: Reads the target path or position. Uses `tf2_ros.TransformListener` to check the `base_link` position relative to the map/odom frame in real time. Computes the error and generates a desired force vector using PID.
- Inputs: Dynamic TF2 tree, target path.
- Outputs: `/marines_rov/cmd_wrench` (`geometry_msgs/Wrench` - desired forces $F_x, F_y, F_z$ and torque $M_z$).

**6. rov_actuation (Hardware Layer / Mixer)**
- Type: `ament_python`
- Task: Implements the Thrust Allocation Matrix (TAM). Takes the overall force request and distributes it across the robot's individual thrusters.
- Portability: In simulation, the output is published to Stonefish. On the real robot, this package sends packets directly to the motor controllers (e.g. via I2C, PWM, or CAN bus).
- Inputs: `/marines_rov/cmd_wrench`
- Outputs: `/thrusters/STONEFISH` (in simulation) or a direct hardware interface.

**7. rov_mission (Mission Management / Top-Level Logic)**
- Type: `ament_python`
- Task: The robot's operational brain (state machine / behavior tree). Responsible for the event sequence: arm thrusters -> dive -> start mapping -> send goal -> monitor state -> return.

**8. rov_bringup (Integration and Orchestration)**
- Type: Top-level meta-package.
- Task: Contains no source code; groups launch files and RViz2 configurations. Allows the whole ecosystem to be started with a single command.

## 5. Required Packages and Environment Modifications (Dockerfile)

The project relies on advanced ROS2 navigation and perception libraries. For the containerized development environment to build the workspace correctly, the Dockerfile must be modified to install key system dependencies.

**Required external ROS2 packages:**
- `ros-humble-robot-localization` (3D Extended Kalman Filter, EKF)
- `ros-humble-octomap-server` (octree mapping and integration server)
- `ros-humble-octomap-msgs` (ROS2 message definitions for OctoMap structures)
- `ros-humble-joy` & `ros-humble-teleop-twist-joy` (gamepad interface and mapping support)

**Required Dockerfile section:**

Before the source code copy stage and the `colcon build` call, the Dockerfile must include the APT package installation instruction:

```dockerfile
# Update repositories and install dependencies for the perception, localization, and control systems
RUN apt-get update && apt-get install -y \
    ros-humble-robot-localization \
    ros-humble-octomap-server \
    ros-humble-octomap-msgs \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    && rm -rf /var/lib/apt/lists/*
```

## 6. Hardware Porting Strategy

The structure is designed so that moving from the simulator to a physical robot running, e.g., on a single-board computer (Raspberry Pi 5 / NVIDIA Jetson) mounted on the ROV requires minimal code changes:

```
    [ SIMULATION MODE ]                              [ PHYSICAL ROBOT MODE ]
  ┌───────────────────────┐                        ┌───────────────────────┐
  │  package:              │                        │  package:              │
  │  rov_simulation       │                        │  hardware_drivers     │
  └──────────┬────────────┘                        └──────────┬────────────┘
             │ (Raw sensors)                                  │ (Real sensors)
             v                                                v
┌──────────────────────────────────────────────────────────────────────────────────┐
│                                Shared System Core                                │
│  [rov_localization (EKF)] -> [rov_mapping (OctoMap)] -> [rov_navigation (RRT*)]  │
│                                       │                                          │
│                                       v                                          │
│                            [rov_control (PID with TF2)]                          │
└───────────────────────────────────────┬──────────────────────────────────────────┘
                                        │ (Requested Wrench)
                                        v
                          ┌───────────────────────────┐
                          │ package: rov_actuation    │
                          └─────────────┬─────────────┘
                                        │
                 ┌──────────────────────┴──────────────────────┐
                 ▼ (Choice of runtime environment)              ▼
      [ Stonefish Simulator ]                         [ Physical thrusters ]
      (Float64MultiArray to .scn)                     (PWM / I2C / CAN interface)
```

- **Disconnecting the simulation:** In the top-level launch file of the `rov_bringup` package, set the `use_sim` flag to `false`. This prevents the `rov_simulation` package from being launched.
- **Connecting physical sensors:** Insert the hardware vendor's nodes in its place (e.g. a micro-ROS package for the IMU, serial-port DVL drivers, and the `zed-ros2-wrapper` driver for the physical camera). The only requirement is remapping the output topics so they reach `rov_localization` and `rov_mapping`.
- **Adapting the thruster allocator:** In the `rov_actuation` package, replace the publishing class. Instead of sending the thrust vector to the `/thrusters/STONEFISH` topic, import a hardware bus library (e.g. `smbus`/`PCA9685` for Python) and send signals directly to the thruster controller pins. The entire localization, 3D mapping, and path planning layer remains untouched.
