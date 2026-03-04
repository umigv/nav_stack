# nav_bringup
Launch files and shared configuration for the navigation stack. Frame IDs and other system-wide constants are defined
in `nav_bringup/global_config.py`.


## core.launch.py
Launches core functionalities used across the rest of the launch files.

```
ros2 launch nav_bringup core.launch.py
```

### Foxglove Bridge
Foxglove bridge is always started on `ws://localhost:8765`. Connect Foxglove Studio to this address to visualize the robot.

### Subscribed Topics
- `teleop_cmd_vel` (`geometry_msgs/Twist`) - Joystick velocity
- `recovery_cmd_vel` (`geometry_msgs/Twist`) - Recovery velocity
- `nav_cmd_vel` (`geometry_msgs/Twist`) - Nav velocity

### Published Topics
- `cmd_vel` (`geometry_msgs/Twist`) - Multiplexed output velocity
- `state` (`std_msgs/msg/String`) - State (`normal`, `ramp` or `recovery`)

### Services
- `state/set_recovery` (`std_srvs/SetBool`) - Set whether we are in recovery mode
- `state/set_ramp` (`std_srvs/SetBool`) - Set whether we are in ramp mode

### Velocity Multiplexing
| Priority | Topic | Source | Timeout |
|---|---|---|---|
| 3 | `teleop_cmd_vel` | Joystick | 0.5s |
| 2 | `recovery_cmd_vel` | Recovery system | 0.5s |
| 1 | `nav_cmd_vel` | Autonomy | 0.5s |

If a higher-priority source stops publishing, control falls back to the next source after 0.5s.


## sensors.launch.py
Launches sensor drivers and static TF transforms. Supports a `simulation` mode that replaces real hardware with 
simulators.

```
ros2 launch nav_bringup sensors.launch.py [simulation:=true]
```

### Parameters
- `simulation`: Launch sensor and occupancy grid simulators instead of real hardware drivers, default `false`

### Published Topics (Hardware)
- `imu/raw` (`sensor_msgs/Imu`) - Raw IMU data from VectorNav
- `gps/raw` (`sensor_msgs/NavSatFix`) - Raw GPS fix from GPS receiver

### Published Topics (Simulation)
- `imu/raw` (`sensor_msgs/Imu`) - Simulated IMU with Gaussian noise
- `gps/raw` (`sensor_msgs/NavSatFix`) - Simulated GPS with noise and OU drift
- `enc_vel/raw` (`geometry_msgs/TwistWithCovarianceStamped`) - Simulated encoder velocity with noise and OU drift
- `odom/ground_truth` (`nav_msgs/Odometry`) - Noiseless true pose in `map` frame
(`child_frame_id = base_link_ground_truth`)
- `occ_grid` (`nav_msgs/OccupancyGrid`) - Robot-centric occupancy grid from static obstacle map
- `occupancy_grid/ground_truth` (`nav_msgs/OccupancyGrid`) - Full static obstacle map (latched)

### Broadcasted TF Frames
- `base_link` → `imu_link`
- `base_link` → `gps_link`
- `map` → `base_link_ground_truth` (simulation mode only) - Noiseless true robot pose


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
