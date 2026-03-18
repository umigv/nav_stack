# nav_bringup
Launch files and configuration for the navigation stack.


## Modes
| Mode | Sensors | `odom`→`base_link` | `map`→`odom` | /goal source | `course` required |
|---|---|---|---|---|---|
| `autonav` | IMU + GPS | EKF | EKF | autonav_goal_selection | yes |
| `autonav_sim` | simulated | EKF | EKF | autonav_goal_selection | yes |
| `self_drive` | IMU | EKF | identity | CV | no |
| `self_drive_sim` | simulated | EKF | identity | CV | yes |
| `nav_test` | none | enc_odom | identity | manual | no |


### Course Configuration
Frame IDs and node parameters are defined in `nav_bringup/config/`. 
To configure a new course, add a subfolder under `nav_bringup/courses/` containing:
- `gps.json` — GPS datum and waypoints
- `map.json` — simulation obstacle map

Courses can be generated using the [course creation tool](https://github.com/umigv/course_creation_tool). The `default`
course is used when no `course` argument is provided. See `nav_bringup/courses/default/` for the expected schema.


## base.launch.py
Launches the base stack required for all operation modes: core, sensors, and localization.

```
ros2 launch nav_bringup base.launch.py mode:=<mode> [course:=<course>]
```

### Parameters
- `mode`: Operation mode, passed through to sensors.launch.py and localization.launch.py (required)
- `course`: Course profile, passed through to sensors.launch.py and localization.launch.py, default `default` (required 
for `autonav`, `autonav_sim`, `self_drive_sim`)


## core.launch.py
Launches core functionalities of the stack.

```
ros2 launch nav_bringup core.launch.py
```

### Robot State Publisher
Loads `marvin_description/urdf/marvin.xacro` and publishes TF transforms for all robot links.

### Foxglove Bridge
Foxglove bridge is always started on `ws://localhost:8765`. Connect Foxglove Studio to this address to visualize the 
robot.

### Subscribed Topics
- `teleop_cmd_vel` (`geometry_msgs/Twist`) - Joystick velocity
- `recovery_cmd_vel` (`geometry_msgs/Twist`) - Recovery velocity
- `nav_cmd_vel` (`geometry_msgs/Twist`) - Nav velocity

### Published Topics
- `robot_description` (`std_msgs/String`) - URDF robot description
- `cmd_vel` (`geometry_msgs/Twist`) - Multiplexed output velocity
- `state` (`std_msgs/msg/String`) - State (`normal`, `ramp` or `recovery`)

### Services
- `state/set_recovery` (`std_srvs/SetBool`) - Set whether we are in recovery mode
- `state/set_ramp` (`std_srvs/SetBool`) - Set whether we are in ramp mode

### Broadcasted TF Frames
See `marvin_description` for the full list of published frames.

### Velocity Multiplexing
| Priority | Topic | Source | Timeout |
|---|---|---|---|
| 3 | `teleop_cmd_vel` | Joystick | 0.5s |
| 2 | `recovery_cmd_vel` | Recovery system | 0.5s |
| 1 | `nav_cmd_vel` | Autonomy | 0.5s |

If a higher-priority source stops publishing, control falls back to the next source after 0.5s.


## sensors.launch.py
Launches sensor drivers

```
ros2 launch nav_bringup sensors.launch.py mode:=<mode> [course:=<course>]
```

### Parameters
- `mode`: Operation mode (required)
- `course`: Course profile in `courses/` to load map and GPS datum from, default `default` (required for `autonav_sim`, 
`self_drive_sim`)

### Published Topics (Hardware)
- `imu/raw` (`sensor_msgs/Imu`) - Raw IMU data from VectorNav (`autonav`, `self_drive`)
- `gps/raw` (`sensor_msgs/NavSatFix`) - Raw GPS fix from GPS receiver (`autonav` only)

### Subscribed Topics (Simulation)
- `cmd_vel` (`geometry_msgs/Twist`) - Velocity the robot is commanded to move in

### Published Topics (Simulation)
- `imu/raw` (`sensor_msgs/Imu`) - Simulated IMU with Gaussian noise
- `gps/raw` (`sensor_msgs/NavSatFix`) - Simulated GPS with noise and OU drift
- `enc_vel/raw` (`geometry_msgs/TwistWithCovarianceStamped`) - Simulated encoder velocity with noise and OU drift
- `odom/ground_truth` (`nav_msgs/Odometry`) - Noiseless true pose in `map` frame
(`child_frame_id = base_link_ground_truth`)
- `occupancy_grid/raw` (`nav_msgs/OccupancyGrid`) - Robot-centric occupancy grid from static obstacle map
- `occupancy_grid/ground_truth` (`nav_msgs/OccupancyGrid`) - Full static obstacle map (latched)

### Broadcasted TF Frames (Simulation)
- `map` → `base_link_ground_truth` - Noiseless true robot pose


## localization.launch.py
Launches localization

```
ros2 launch nav_bringup localization.launch.py mode:=<mode> [course:=<course>]
```

### Parameters
- `mode`: Operation mode (required)
- `course`: Course profile in `courses/` to load GPS datum from, default `default` (required for `autonav`, 
`autonav_sim`)

### Subscribed Topics
- `imu/raw` (`sensor_msgs/Imu`) - Raw IMU data (autonav*, self_drive* only)
- `gps/raw` (`sensor_msgs/NavSatFix`) - Raw GPS fix (autonav* only)
- `enc_vel/raw` (`geometry_msgs/TwistWithCovarianceStamped`) - Encoder velocity

### Published Topics
- `odom/local` (`nav_msgs/Odometry`) - Local odometry in the odom frame
- `odom/global` (`nav_msgs/Odometry`) - Global odometry in the map frame (autonav* only)

### Broadcasted TF Frames
- `odom` → `base_link`
- `map` → `odom`

### Services
- `fromLL` (`robot_localization/FromLL`) - Converts GPS latitude/longitude to a map-frame point (autonav* only)


## navigation.launch.py
Launches the navigation stack.

```
ros2 launch nav_bringup navigation.launch.py mode:=<mode>
```

### Parameters
- `mode`: Operation mode (required)


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
