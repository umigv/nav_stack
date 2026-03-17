# UMARV Navigation Stack 2025-2026

## Warning: Intellisense
When building, make sure you include `--symlink-install`. Otherwise, when control clicking a dependency in VSCode, it 
will take you to a copy of the dependency, instead of the actual source file. For example:
```bash
colcon build --symlink-install
source install/setup.bash
```

## Dependencies
You can install all dependencies by running
```bash
./scripts/setup.sh
```

## Running the stack
Run each of these commands in separate terminals:
```bash
ros2 launch nav_bringup base.launch.py mode:=<mode> [course:=<course>]
```
```bash
ros2 launch nav_bringup teleop.launch.py controller:=<ps4/xbox>
```
and / or
```bash
ros2 launch nav_bringup navigation.launch.py mode:=<mode> [course:=<course>]
```

### Modes
| Mode | Sensors | `odom`→`base_link` | `map`→`odom` | /goal source | `course` required |
|---|---|---|---|---|---|
| `autonav` | hardware | EKF | EKF | autonav_goal_selection | yes |
| `autonav_sim` | simulated | EKF | EKF | autonav_goal_selection | yes |
| `self_drive` | hardware | EKF | identity | CV | no |
| `self_drive_sim` | simulated | EKF | identity | CV | yes |
| `nav_test` | none | enc_odom | identity | manual | no |

### Configuration
Frame IDs and node parameters are defined in `nav_bringup/config/`. 
To configure a new course, add a subfolder under `nav_bringup/courses/` containing:
- `gps.json` — GPS datum and waypoints
- `map.json` — simulation obstacle map

Courses can be generated using the [course creation tool](https://github.com/umigv/course_creation_tool). The `default`
course is used when no `course` argument is provided. See `nav_bringup/courses/default/` for the expected schema.

## Visualization
You can either 
- Install [Foxglove Studio](https://foxglove.dev/download) and open a new Foxglove WebSocket connection at
`ws://localhost:8765`
- Run `rviz2` in a new terminal and add the topics you want to visualize
