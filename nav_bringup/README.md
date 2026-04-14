# nav_bringup
Launch files and configuration for the navigation stack.


## Modes
| Mode | `odom`→`base_link` | `map`→`odom` | /goal source |
|---|---|---|---|
| `autonav` | EKF | EKF | autonav_goal_selection |
| `self_drive` | EKF | identity | CV |
| `nav_test` | enc_odom | identity | manual |

Pass `simulation:=true` to use simulated sensors instead of hardware. `course` is required for `autonav` and when 
`simulation:=true`.


### Course Configuration
Frame IDs and node parameters are defined in `nav_bringup/config/`. 
To configure a new course, add a subfolder under `nav_bringup/courses/` containing:
- `gps.json` — GPS datum and waypoints
- `map.json` — simulation obstacle map

Courses can be generated using the [course creation tool](https://github.com/umigv/course_creation_tool). The `default`
course is used when no `course` argument is provided. See `nav_bringup/courses/default/` for the expected schema.


## gps_origin_calculator.launch.py
Computes and records the GPS datum for a course. Run this once with the robot stationary at the start position before
an autonomous run. Starts the VectorNav driver, collects samples for 60–90 seconds, writes the median lat/lon/alt into
the course's `gps.json`, then shuts down both nodes automatically.

```
ros2 launch nav_bringup gps_origin_calculator.launch.py [course:=<course>]
```

### Parameters
- `course`: Course profile in `courses/` whose `gps.json` will be updated, default `default`

### Published Topics
- `gps/raw` (`sensor_msgs/NavSatFix`) - Raw GPS fix from VectorNav INS


## base.launch.py
Launches the base stack required for all operation modes: core, hardware/simulation, and localization.

```
ros2 launch nav_bringup base.launch.py mode:=<mode> [simulation:=true] [course:=<course>]
```

### Parameters
- `mode`: Operation mode, passed through to hardware/simulation and localization launch files (required)
- `simulation`: Use simulation instead of hardware sensors, default `false`
- `course`: Course profile, default `default` (required for `mode:=autonav` or `simulation:=true`)


## core.launch.py
Launches core functionalities of the stack.

```
ros2 launch nav_bringup core.launch.py
```

### Robot State Publisher
Loads `marvin_description/urdf/marvin.xacro` and publishes TF transforms for all robot links.

### Subscribed Topics
- `teleop_cmd_vel` (`geometry_msgs/Twist`) - Joystick velocity
- `recovery_cmd_vel` (`geometry_msgs/Twist`) - Recovery velocity
- `nav_cmd_vel` (`geometry_msgs/Twist`) - Nav velocity

### Published Topics
- `robot_description` (`std_msgs/String`) - URDF robot description
- `cmd_vel` (`geometry_msgs/Twist`) - Multiplexed output velocity
- `state` (`std_msgs/msg/String`) - State (`normal`, `no_mans_land` or `recovery`)

### Services
- `state/set_recovery` (`std_srvs/SetBool`) - Set whether we are in recovery mode
- `state/set_no_mans_land` (`std_srvs/SetBool`) - Set whether we are in no mans land mode

### Broadcasted TF Frames
See `marvin_description` for the full list of published frames.

### Velocity Multiplexing
| Priority | Topic | Source | Timeout |
|---|---|---|---|
| 3 | `teleop_cmd_vel` | Joystick | 0.5s |
| 2 | `recovery_cmd_vel` | Recovery system | 0.5s |
| 1 | `nav_cmd_vel` | Autonomy | 0.5s |

If a higher-priority source stops publishing, control falls back to the next source after 0.5s.


## hardware.launch.py
Launches hardware drivers

```
ros2 launch nav_bringup hardware.launch.py mode:=<mode>
```

### Parameters
- `mode`: Operation mode (required)

### Published Topics
- `imu/raw` (`sensor_msgs/Imu`) - Raw IMU data from VectorNav (`autonav`, `self_drive`)
- `gps/raw` (`sensor_msgs/NavSatFix`) - GPS/INS fix from VectorNav (`autonav`, `self_drive`)
- `ins_vel/raw` (`geometry_msgs/TwistWithCovarianceStamped`) - INS body-frame velocity (`autonav`, `self_drive`)
- `vectornav/raw/ins_status` (`std_msgs/UInt16`) - INS tracking status bitfield (`autonav`, `self_drive`)


## simulation.launch.py
Launches simulated sensors. Included by `base.launch.py` when `simulation:=true`.

```
ros2 launch nav_bringup simulation.launch.py [course:=<course>]
```

### Parameters
- `course`: Course profile in `courses/` to load map and GPS datum from, default `default`

### Subscribed Topics
- `cmd_vel` (`geometry_msgs/Twist`) - Velocity the robot is commanded to move in

### Published Topics
- `imu/raw` (`sensor_msgs/Imu`) - Simulated IMU with Gaussian noise
- `gps/raw` (`sensor_msgs/NavSatFix`) - Simulated GPS with noise and OU drift
- `enc_vel/raw` (`geometry_msgs/TwistWithCovarianceStamped`) - Simulated encoder velocity with noise and OU drift
- `odom/ground_truth` (`nav_msgs/Odometry`) - Noiseless true pose in `map` frame 
(`child_frame_id = base_link_ground_truth`)
- `occupancy_grid/raw` (`nav_msgs/OccupancyGrid`) - Robot-centric occupancy grid from static obstacle map
- `occupancy_grid/ground_truth` (`nav_msgs/OccupancyGrid`) - Full static obstacle map (latched)

### Broadcasted TF Frames
- `map` → `base_link_ground_truth` - Noiseless true robot pose


## localization.launch.py
Launches localization

```
ros2 launch nav_bringup localization.launch.py mode:=<mode> [course:=<course>]
```

### Localization Strategy
Uses the standard `robot_localization` dual-EKF pattern:

**Local EKF (`ekf_local`)** - estimates the `odom` → `base_link` transform
- Fuses encoder velocity (vx, vy) and IMU yaw rate (wz)
- Odom origin is where the robot started
- Drift accumulates over time but is smooth and continuous
- Optionally replaced by simple encoder odometry integration (enc_odom_publisher)

**Navsat Transform (`navsat_transform`)** - converts GPS fixes into map-frame odometry
- Receives raw GPS fixes and IMU heading
- Outputs `odom/gps`: GPS position expressed in the map frame

**Global EKF (`ekf_global`)** - estimates the `map` → `odom` transform
- Fuses encoder velocity (vx, vy), IMU yaw + yaw rate, and GPS position (x, y from `odom/gps`)
- Map frame is in ENU where origin is the datum
- Corrects accumulated local drift by anchoring to GPS

### Parameters
- `mode`: Operation mode (required)
- `course`: Course profile in `courses/` to load GPS datum from, default `default` (required for `autonav`)

### Subscribed Topics
- `imu/raw` (`sensor_msgs/Imu`) - Raw IMU data (autonav, self_drive only)
- `gps/raw` (`sensor_msgs/NavSatFix`) - Raw GPS fix (autonav only)
- `enc_vel/raw` (`geometry_msgs/TwistWithCovarianceStamped`) - Encoder velocity

### Published Topics
- `odom/local` (`nav_msgs/Odometry`) - Local odometry in the odom frame
- `odom/global` (`nav_msgs/Odometry`) - Global odometry in the map frame (autonav only)

### Broadcasted TF Frames
- `odom` → `base_link`
- `map` → `odom`

### Services
- `fromLL` (`robot_localization/FromLL`) - Converts GPS latitude/longitude to a map-frame point (autonav only)


## navigation.launch.py
Launches the navigation stack.

```
ros2 launch nav_bringup navigation.launch.py mode:=<mode> [course:=<course>]
```

### Parameters
- `mode`: Operation mode (required)
- `course`: Course profile in `courses/` to load waypoints from, default `default` (required for `autonav`)

### Subscribed Topics
- `goal` (`geometry_msgs/PointStamped`) - Goal for path planning (`self_drive`, `nav_test` only)
- `occupancy_grid/raw` (`nav_msgs/OccupancyGrid`) - Raw occupancy grid from CV
- `odom/local` (`nav_msgs/Odometry`) - Odometry from localization

### Published Topics
- `goal` (`geometry_msgs/PointStamped`) - Goal for path planning (`autonav` only)
- `gps_waypoint` (`geometry_msgs/PointStamped`) - Current GPS waypoint target (`autonav` only)
- `nav_cmd_vel` (`geometry_msgs/Twist`) - Velocity command consumed by twist_mux

### Service Clients
- `fromLL` (`robot_localization/FromLL`) - Converts GPS coordinates to map-frame points; called at startup by 
autonav_goal_selection (`autonav` only)


## teleop.launch.py
Launches joystick teleoperation

```
ros2 launch nav_bringup teleop.launch.py controller:=<controller>
```

### Parameters
- `controller`: Controller profile (`xbox` or `ps4`), required

### Controller Mappings
For both Xbox and PS4:
- Left joystick - linear motion
- Right joystick - turning
- Right shoulder button (RB / R1) - enable
- Left shoulder button (LB / L1) - turbo

### Published Topics
- `joy` (`sensor_msgs/Joy`) - Raw joystick input
- `teleop_cmd_vel` (`geometry_msgs/Twist`) - Joystick velocity command


## visualization.launch.py
Launches visualization tools for Foxglove Studio.

```
ros2 launch nav_bringup visualization.launch.py
```

### Foxglove Bridge
Starts Foxglove bridge on `ws://localhost:8765`.

### Occupancy Grid Voxel Visualization
Subscribes to `occupancy_grid` and republishes it as a `foxglove_msgs/VoxelGrid` on `occupancy_grid/voxels` for 3D voxel
visualization in Foxglove Studio.

### Subscribed Topics
- `occupancy_grid` (`nav_msgs/OccupancyGrid`) - Occupancy grid input

### Published Topics
- `occupancy_grid/voxels` (`foxglove_msgs/VoxelGrid`) - Voxel representation of the occupancy grid
