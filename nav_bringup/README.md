# nav_bringup
Launch files and configuration for the navigation stack. See the root README for configuration and course data setup.


## base.launch.py
Launches the base stack required for all operation modes: core, sensors, and localization.

```
ros2 launch nav_bringup base.launch.py [simulation:=true] [use_enc_odom:=true] [course:=<course>]
```

### Parameters
- `simulation`: Pass through to `sensors.launch.py`, default `false`
- `use_enc_odom`: Pass through to `localization.launch.py`, default `false`
- `course`: Pass through to `sensors.launch.py` and `localization.launch.py`, default `default`


## core.launch.py
Launches core functionalities of the stack.

```
ros2 launch nav_bringup core.launch.py
```

### Robot State Publisher
Loads `marvin_description/urdf/marvin.xacro` and publishes TF transforms for all robot links.

### Foxglove Bridge
Foxglove bridge is always started on `ws://localhost:8765`. Connect Foxglove Studio to this address to visualize the robot.

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
Launches sensor drivers. Supports a `simulation` mode that replaces real hardware with simulators.

```
ros2 launch nav_bringup sensors.launch.py [simulation:=true]
```

### Parameters
- `simulation`: Launch sensor and occupancy grid simulators instead of real hardware drivers, default `false`
- `course`: Course profile in `courses/` to load map and GPS datum from, default `default`

### Published Topics (Hardware)
- `imu/raw` (`sensor_msgs/Imu`) - Raw IMU data from VectorNav
- `gps/raw` (`sensor_msgs/NavSatFix`) - Raw GPS fix from GPS receiver

### Published Topics (Simulation)
- `imu/raw` (`sensor_msgs/Imu`) - Simulated IMU with Gaussian noise
- `gps/raw` (`sensor_msgs/NavSatFix`) - Simulated GPS with noise and OU drift
- `enc_vel/raw` (`geometry_msgs/TwistWithCovarianceStamped`) - Simulated encoder velocity with noise and OU drift
- `odom/ground_truth` (`nav_msgs/Odometry`) - Noiseless true pose in `map` frame
(`child_frame_id = base_link_ground_truth`)
- `occupancy_grid/raw` (`nav_msgs/OccupancyGrid`) - Robot-centric occupancy grid from static obstacle map
- `occupancy_grid/ground_truth` (`nav_msgs/OccupancyGrid`) - Full static obstacle map (latched)

### Broadcasted TF Frames
- `map` → `base_link_ground_truth` (simulation mode only) - Noiseless true robot pose


## localization.launch.py
Launches localization

```
ros2 launch nav_bringup localization.launch.py [use_enc_odom:=true]
```

### Parameters
- `use_enc_odom`: Use encoder odometry integration instead of EKF for local odometry, default `false`
- `course`: Course profile in `courses/` to load GPS datum from, default `default`

### Subscribed Topics
- `imu/raw` (`sensor_msgs/Imu`) - Raw IMU data
- `gps/raw` (`sensor_msgs/NavSatFix`) - Raw GPS fix
- `enc_vel/raw` (`geometry_msgs/TwistWithCovarianceStamped`) - Encoder velocity

### Published Topics
- `odom/local` (`nav_msgs/Odometry`) - Local odometry in the odom frame
- `odom/global` (`nav_msgs/Odometry`) - Global odometry in the map frame

### Broadcasted TF Frames
- `odom` → `base_link`
- `map` → `odom`

### Services
- `fromLL` (`robot_localization/FromLL`) - Converts GPS latitude/longitude to a map-frame point


## navigation.launch.py
Launches the navigation stack.

```
ros2 launch nav_bringup navigation.launch.py
```


## teleop.launch.py
Launches joystick teleoperation

```
ros2 launch nav_bringup teleop.launch.py controller:=<controller>
```

### Parameters
- `controller`: Controller profile (`xbox` or `ps4`), required
- `joystick_dev`: Joystick device path, default `/dev/input/js0`

### Controller Mappings
For both Xbox and PS4:
- Left joystick - linear motion
- Right joystick - turning
- Right shoulder button (RB / R1) - enable
- Left shoulder button (LB / L1) - turbo

### Published Topics
- `joy` (`sensor_msgs/Joy`) - Raw joystick input
- `teleop_cmd_vel` (`geometry_msgs/Twist`) - Joystick velocity command
