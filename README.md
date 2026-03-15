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

## Configuration
Frame IDs and node parameters are defined in `nav_bringup/config/`. 
To configure a new course, add a subfolder under `nav_bringup/courses/` containing:
- `gps.json` — GPS datum and waypoints
- `map.json` — simulation obstacle map

Courses can be generated using the [course creation tool](https://github.com/umigv/course_creation_tool). The `default` 
course is used when no `course` argument is provided.

## Running the stack
Run each of these commands in separate terminals:
```bash
ros2 launch nav_bringup base.launch.py [simulation:=true] [use_enc_odom:=true] [course:=<course>]
```
```bash
ros2 launch nav_bringup teleop.launch.py controller:=<ps4/xbox> # teleop
```
# and / or
```bash
ros2 launch nav_bringup navigation.launch.py [course:=<course>] # autonav
```

## Visualization
You can either 
- Install [Foxglove Studio](https://foxglove.dev/download) and open a new Foxglove WebSocket connection at
`ws://localhost:8765`
- Run `rviz2` in a new terminal and add the topics you want to visualize
