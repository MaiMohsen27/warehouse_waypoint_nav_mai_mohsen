# warehouse_waypoint_nav

Autonomous mission execution for a **TurtleBot3 Burger** operating inside a simulated warehouse, built on the **ROS 2 Nav2** stack. The robot maps the environment, localizes itself, and executes an ordered waypoint mission with real-time status visualization in RViz.
 
---
## Table of Contents
 
1. [Project Overview and Mission](#1-project-overview-and-mission)
2. [Repository Structure](#2-repository-structure)
3. [Build Instructions](#3-build-instructions)
4. [Usage and Testing](#4-usage-and-testing)
   - [4.1 Launch the Simulation](#41-launch-the-simulation)
   - [4.2 Build the Map (SLAM)](#42-build-the-map-slam)
   - [4.3 Localization (AMCL)](#43-localization-amcl)
   - [4.4 Navigation (Nav2 Stack)](#44-navigation-nav2-stack)
   - [4.5 Run the Waypoint Mission](#45-run-the-waypoint-mission)
5. [Warehouse Waypoint Mission](#5-warehouse-waypoint-mission)
   - [5.1 Waypoint Names, Positions, and Orientations](#51-waypoint-names-positions-and-orientations)
   - [5.2 Mission Route)](#52-mission-route)
   - [5.3 RViz2 Waypoint Marker Behavior](#53-rviz2-waypoint-marker-behavior)
   - [5.4 Required Terminal Output](#44-navigation-nav2-stack)
6. [Problems Encountered and Their Solutions](#6-problems-encountered-and-their-solutions)
7. [Full Demo](#7-full-demo)
---
## 1. Project Overview and Mission
 
This repository delivers an end-to-end autonomous navigation pipeline for a TurtleBot3 Burger operating in a simulated warehouse. The system is responsible for:
 
- Mapping the warehouse environment using **SLAM Toolbox**.
- Localizing the robot within the saved map using **AMCL**.
- Planning and executing motion via the full **Nav2** stack (Planner, Controller, Behavior, and BT Navigator servers).
- Executing a fixed, ordered mission across four named locations, with a timed hold at one station.
- Publishing live RViz markers that reflect which waypoint is the current navigation target.

### Required Mission
1. Start at the Charging Station (Home).
2. Navigate to the Loading Station.
3. Wait there exactly 30 seconds.
4. Navigate to the Storage Area.
5. Navigate to the Shipping Station.
6. Return to the Charging Station (Home) and report the mission is complete.
---
## 2. Repository Structure
 
```
warehouse_waypoint_nav_mai_mohsen/
├── turtlebot3_gazebo/ #cloned
│   ├── launch/
│   ├── models/ #.sdf files max laser range are edited to 10.0 m
│   ├── params/
│   ├── rviz/
│   └── urdf/
├── warehouse_world/ #cloned 
│   ├── config/
│   ├── launch/
│   │   └── warehouse_storage_launch.launch.py #edited
│   ├── models/
│   ├── worlds/
│   ├── CMakeLists.txt
│   └── package.xml
├── slam_toolbox_demo/
│   ├── config/
│   │   └── slam_toolbox_online_async.yaml
│   ├── launch/
│   │   └── slam_toolbox_online_async.launch.py
│   ├── map/
│   │   ├── turtlebot3_world_map.yaml
│   │   └── turtlebot3_world_map.pgm
│   ├── rviz/
│   │   └── mapping_configs.rviz
│   ├── CMakeLists.txt
│   └── package.xml
├── robot_localization/
│   ├── config/
│   │   └── amcl.yaml
│   ├── launch/
│   │   └── amcl.launch.py
│   ├── map/
│   │   ├── turtlebot3_world_map.yaml
│   │   └── turtlebot3_world_map.pgm
│   ├── rviz/
│   │   └── amcl_configs.rviz
│   ├── CMakeLists.txt
│   └── package.xml
├── robot_navigation/
│   ├── config/
│   │   ├── planner_server.yaml
│   │   ├── controller_server.yaml
│   │   ├── behavior_server.yaml
│   │   └── bt_navigator.yaml
│   ├── launch/
│   │   └── nav2_bringup.launch.py
│   ├── map/
│   │   ├── turtlebot3_world_map.yaml
│   │   └── turtlebot3_world_map.pgm
│   ├── rviz/
│   │   └── nav2_configs.rviz
│   ├── CMakeLists.txt
│   └── package.xml
├── warehouse_waypoint/
│   ├── resource/
│   │   └── warehouse_waypoint
│   ├── warehouse_waypoint/
│   │   └── nodes/
│   │       └── mission_node.py
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
├── images/
└── README.md
```
| Layer | Responsibility | Key Packages |
|---|---|---|
| Simulation | Spawns the TurtleBot3 Burger in the warehouse world | `turtlebot3_gazebo`, `warehouse_world` |
| Mapping | Builds the occupancy grid map | `slam_toolbox_demo` |
| Localization | Estimates robot pose within the map | `robot_localization` |
| Navigation | Global/local planning, obstacle avoidance, recovery behaviors | `robot_navigation` |
| Mission Logic | Sends ordered goals, handles the timed wait, reports failures | `warehouse_waypoint` |
| Visualization | Renders map, TF, costmaps, and waypoint markers | RViz2 |

---
## 3. Build Instructions
 
1. **Create a ROS 2 workspace** with a `src` folder in it.
   ```bash
   mkdir -p ros2_ws/src
   ```
   
2.  **Clone this repository** into your ROS 2 workspace `src` folder:
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/MaiMohsen27/warehouse_waypoint_nav_mai_mohsen.git
    ```

3. **Build the package:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select robot_localization
   source install/setup.bash
   ```

---

## 4. Usage and Testing
 
### 4.1 Launch the Simulation
 
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch warehouse_world warehouse_storage_launch.launch.py
```
 
Verify before proceeding:
- **Confirm `/scan` and `/odom` are available:**
  ```bash
  ros2 topic list
  ```

- **Check LiDAR:**
  ```bash
  ros2 topic echo /scan --once
  ```
  The output should look like this:
  ```
  header:
  stamp:
    sec: 94
    nanosec: 200000000
  frame_id: base_scan
  angle_min: 0.0
  angle_max: 6.28000020980835
  angle_increment: 0.01749303564429283
  time_increment: 0.0
  scan_time: 0.0
  range_min: 0.11999999731779099
  range_max: 10.0
  ```
  
- **Check odometry:**
  ```bash
  ros2 topic echo /odom --once
  ```
  The output should look like this:
  ```
  header:
  stamp:
    sec: 343
    nanosec: 460000000
  frame_id: odom
  child_frame_id: base_footprint
  pose:
    pose:
      position:
        x: -2.2887103058319675e-12
        y: 1.5386449147888426e-25
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: -1.0655300440780967e-11
        w: 1.0
  ```
[Demo](https://drive.google.com/file/d/1kYlkbwwyZgKSAGLa7eBTDhSYVEfOeQ-i/view?usp=drive_link)

### 4.2 Build the Map (SLAM)

In a new terminal:
- Run SLAM toolbox to build a map:
  ```bash
  source ~/workspaces/ros2_ws/install/setup.bash
  ros2 launch slam_toolbox_demo slam_toolbox_online_async.launch.py
  ```
In a new terminal:
- Open RViz2:
  ```bash
  source ~/workspaces/ros2_ws/install/setup.bash
  rviz2
  ```
- Open `mapping_configs.rviz` in RViz2.

<img width="1920" height="772" alt="3-mapping1" src="https://github.com/user-attachments/assets/87a1e865-94a0-4c57-9193-b8daddf39c39" />

- View the TF tree:
  ```bash
  ros2 run tf2_tools view_frames
  ```
  
<img width="812" height="468" alt="7-TF_tree_mapping" src="https://github.com/user-attachments/assets/a5b8ae42-b27d-4084-9b22-a612c9d7e3c9" />

In a new terminal:
- Drive the robot using teleop_keyboard:
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```
 
Drive through every aisle, corner, and open area until the map is complete, with no duplicated walls or unexplored gaps.

<img width="777" height="637" alt="4-mapping2" src="https://github.com/user-attachments/assets/6e9e58e2-fc75-49ca-895a-b0ec8879d030" />

- When you are done, save the map:
In a new terminal:
  ```bash
  cd ~/workspaces/ros2_ws/src/slam_toolbox_demo/map
  ros2 run nav2_map_server map_saver_cli -f turtlebot3_world_map
  ```
To view the saved map, open `turtlebot3_world_map.pgm`. Make sure you have pgm viewer extension.

<img width="1107" height="501" alt="5-map_pgm" src="https://github.com/user-attachments/assets/5b174a0e-d829-485f-842c-9e78e74bb73c" />

- Test the map:
Stop `slam_toolbox_online_async.launch.py` first. Then:
  ```bash
  cd ~/workspaces/ros2_ws/src/slam_toolbox_demo/map
  ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=turtlebot3_world_map.yaml
  ```
In a new terminal:
  ```bash
  cd ~/workspaces/ros2_ws/src/slam_toolbox_demo/map
  ros2 run nav2_util lifecycle_bringup map_server
  ```
In a new terminal:
  ```bash
  cd ~/workspaces/ros2_ws/src/slam_toolbox_demo/map
  rviz2
  ```
  Add `Map` display
  Under QoS Settings:
  ```
  Durability Policy = Transient Local
  ```
It should display as it was built.

<img width="880" height="662" alt="6-saved_map" src="https://github.com/user-attachments/assets/d685144b-fbab-42b7-8677-013e81b570a5" />

[Demo](https://drive.google.com/file/d/1d1RK9sMmvzxc9XidUu9wb9OjGUcIPvEi/view?usp=drive_link)

### 4.3 Localization (AMCL)

- **Launch the Gazebo simulation:**
   In a new terminal:
   ```bash
   ros2 launch warehouse_world warehouse_storage_launch.launch.py
   ```

- **Launch map_server, AMCL, and lifecycle_manager:**
   In a new terminal:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch robot_localization amcl.launch.py
   ```
- **Launch RViz:**
   In a new terminal:
   ```bash
   source ~/ros2_ws/install/setup.bash
   rviz2
   ```
- Open `amcl_configs.rviz` in RViz2.
   
- Give the robot the **correct initial pose**:
   - Using the `2D Pose Estimate` tool, click on the map where the robot is located.
   - Then, hold and drag the arrow in the direction the robot is facing, and observe.
   - The LiDAR scan should align with the map walls and the particle cloud should converge around the robot.

<img width="975" height="737" alt="8-map_amcl" src="https://github.com/user-attachments/assets/0b6e2df1-4547-46ca-b1b9-b664802e806e" />

- Drive the robot with teleop and observe the particle cloud converge around the robot's true pose:
   ```bash
   ros2 run turtlebot3_teleop teleop_keyboard
   ```

- Confirm the Tf tree:
  ```bash
  ros2 run tf2_tools view_frames
  ```
<img width="886" height="530" alt="9-TF_tree_amcl" src="https://github.com/user-attachments/assets/3b0ebdfd-0d63-4207-ab19-3f8428892b4c" />

- **Confirm `/amcl_pose` updates while the robot moves:**
  ```bash
  ros2 topic echo /amcl_pose --once
  ```
  The output should look like this:
  ```
  header:
  stamp:
    sec: 659
    nanosec: 600000000
  frame_id: map
  pose:
    pose:
      position:
        x: 0.9522368008854546
        y: -0.24267514450928243
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: -0.008643751055602718
        w: 0.9999626420860375
  ```
[Demo](https://drive.google.com/file/d/1ob8CEMFuXQmqwha4CWyGYPdTTp2DZzZF/view?usp=drive_link)

### 4.4 Navigation (Nav2 Stack)

- **Launch the Gazebo simulation:**
   In a new terminal:
   ```bash
   ros2 launch warehouse_world warehouse_storage_launch.launch.py
   ```

- **Launch nav2 stack:**
   In a new terminal:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch robot_navigation nav2_bringup.launch.py
   ```
- **Launch RViz:**
   In a new terminal:
   ```bash
   source ~/ros2_ws/install/setup.bash
   rviz2
   ```
- Open `nav2_configs.rviz` in RViz2.

<img width="976" height="731" alt="10-nav2_map" src="https://github.com/user-attachments/assets/3d23c8a9-2e7f-4359-a400-b1a5732e6ea7" />

- Send a Navigation Goal:
   - Using the `2D Goal Pose` tool, Click somewhere empty on the map.
   - Drag to choose the final orientation then release.
   - The robot should go to the exact same location you specified.

<img width="691" height="476" alt="image" src="https://github.com/user-attachments/assets/a7614f1f-112f-4e16-a2f4-8578e49addb5" />

<img width="1366" height="497" alt="image" src="https://github.com/user-attachments/assets/e529ec7b-eabc-474f-a2df-86d9c7b00c47" />

- **Confirm `/cmd_vel` uses `geometry_msgs/msg/Twist`:**
  ```bash
  ros2 topic info -v /cmd_vel
  ```
  The topic type must be:
  ```
  geometry_msgs/msg/Twist
  ```
[Demo](https://drive.google.com/file/d/1TxcNj-X3EeXPIIo-4PbQFrgTKdNA6jZ3/view?usp=drive_link)

### 4.5 Run the Waypoint Mission

- **Launch the Gazebo simulation:**
   In a new terminal:
   ```bash
   ros2 launch warehouse_world warehouse_storage_launch.launch.py
   ```

- **Launch nav2 stack:**
   In a new terminal:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch robot_navigation nav2_bringup.launch.py
   ```
- **Launch RViz:**
   In a new terminal:
   ```bash
   source ~/ros2_ws/install/setup.bash
   rviz2
   ```
- Open `nav2_configs.rviz` in RViz2.
- Add ──► By display type ──► rviz_default_plugins ──► MarkerArray
- **Start the mission:**
   In a new terminal:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 run warehouse_waypoint mission_node.py
   ```
- In MarkerArray, change the topic to `/waypoint_markers`
  
<img width="1366" height="768" alt="image" src="https://github.com/user-attachments/assets/8ca330c2-65db-4b21-842b-463465a84d1e" />


- Now observe Simulation, RViz2, and the terminal for feedback.

[Demo](https://drive.google.com/file/d/1Bf_IglCZ16e1Zo42NxQHbj0_bNoVyAGb/view?usp=drive_link)
---
## 5. Warehouse Waypoint Mission
 
`mission_node.py` uses the Nav2 `NavigateToPose` action to send the robot through four warehouse stations in order, holds at the Loading station for 30 seconds, and publishes a `MarkerArray` so the mission can be visualized in RViz.

### 5.1 Waypoint Names, Positions, and Orientations
 
All poses are in the `map` frame as `(x, y, yaw)`. Yaw is converted to a quaternion via `orientation.z = sin(yaw/2)`, `orientation.w = cos(yaw/2)`.
 
| # | Station  | x (m) | y (m) | yaw (rad) | Quaternion (x, y, z, w) |
|---|----------|------:|------:|----------:|-------------------------|
| 0 | Loading  | 12.0  | 0.0   | 0.0       | (0, 0, 0, 1)            |
| 1 | Storage  | 12.0  | 5.0   | 0.0       | (0, 0, 0, 1)            |
| 2 | Shipping | 3.0   | 5.0   | 0.0       | (0, 0, 0, 1)            |
| 3 | Charging | 0.0   | 0.0   | 0.0       | (0, 0, 0, 1)            |
 
- **Home** and **Charging Station** are the same physical point, `(0.0, 0.0, 0.0)`.
- All four yaw values are currently `0.0`.

### 5.2 Mission Route
 
```
Home (Charging Station) → Loading Station (hold 30s) → Storage Area → Shipping Station → Home (Charging Station)
```
 
1. Start at **Home / Charging Station**.
2. Navigate to **Loading Station** → hold position for **30 seconds**.
3. Navigate to **Storage Area**.
4. Navigate to **Shipping Station**.
5. Navigate back to **Home / Charging Station** → mission complete.

### 5.3 RViz2 Waypoint Marker Behavior
 
Markers are published on `/waypoint_markers` as a `MarkerArray` (one `SPHERE` + one `TEXT_VIEW_FACING` label per station).
 
| Color | Meaning              | Label text        |
|-------|----------------------|--------------------|
| 🔵 Blue  | Inactive waypoint  | `<Name> (Inactive)` |
| 🟢 Green | Active navigation goal | `<Name> (Active)` |

Behavior rules:
- Every station starts **blue**.
- Only one station is green at any given time, which is the current goal the robot is heading/ staying at
- As soon as the mission advances to the next station, the previous station reverts to **blue**.
  
<img width="441" height="270" alt="mission_2" src="https://github.com/user-attachments/assets/68efcfcb-41b2-4a1c-bdca-ace46370dc39" />

### 5.4 Required Terminal Output
 
Expected output during a normal run:
 
```
[INFO] Mission started at Charging Station.
[INFO] Navigating to Loading Station...
[INFO] Waiting for navigate_to_pose action server...
[INFO] Sending goal to loading: x=12.00, y=0.00, yaw=0.00
[INFO] Goal accepted, navigating...
[INFO] Distance remaining: ...
[INFO] Reached Loading Station.
[INFO] Waiting 30s at Loading Station...
[INFO] Navigating to Storage Station...
...
[INFO] Reached Shipping Station.
[INFO] Navigating to Charging Station...
[INFO] Reached Charging Station.
[INFO] Mission complete. Robot returned to Charging Station.
```
 
On failure at any leg:
 
```
[ERROR] Navigation to <Station> Station failed (status=<code>). Mission aborted.
```
 
or, if a goal is rejected outright:
 
```
[ERROR] Goal to <Station> Station was rejected. Mission aborted.
```

<img width="1097" height="509" alt="mission_waiting" src="https://github.com/user-attachments/assets/25310e4a-aaf1-48c3-8273-80e36ba3d126" />

<img width="874" height="436" alt="mission_6" src="https://github.com/user-attachments/assets/f4e6c967-d8a8-4ed7-b723-bf5f26eea44c" />


[Demo](https://drive.google.com/file/d/1puqMhkpV-n5fMkMEhyCLUnPmNgbyfKuM/view?usp=drive_link)
---

## 6. Problems Encountered and Their Solutions
1.
``` 
[tf2_buffer]: Detected jump back in time. Clearing TF buffer.
```
**when**: The problem appeared in Navigation
**why**: Two `ros_gz_bridge` instances ended up running at the same time, each of them published at `/clock` independently. One was the nav2 launch file and the other was the `turtlebot3_gazebo` bridge, started separately inside `spawn_turtlebot3.launch.py` via `turtlebot3_burger_bridge.yaml`. 
**solution**: The fix was to kill all background processes and remove the redundant `ros_gz_bridge` node entirely from `spawn_turtlebot3.launch.py`.

2.
**Inaccurate mapping**
**when**: The problem appeared during SLAM/mapping
**why**: The laser scan wasn't reaching far enough to detect surrounding obstacles, resulting in poor obstacle readings and an inaccurate map.
**solution**: The fix was to increase the laser scan's max range parameter to 10.0 (in .sdf files in `turtlebot3_gazebo/models/`, allowing the LIDAR to properly detect obstacles at the required distances and produce a correct map.

---
## 7. Full Demo

[Full_Demo]()

---
**Author**: Mai Mohsen

