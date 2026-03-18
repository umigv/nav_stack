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
ros2 launch nav_bringup navigation.launch.py mode:=<mode>
```

### Mode and Course Configuration
See [nav_bringup/README.md](src/nav_infrastructure_simple/nav_bringup/README.md) for mode and course configuration 
details.


## Visualization
You can either 
- Install [Foxglove Studio](https://foxglove.dev/download) and open a new Foxglove WebSocket connection at
`ws://localhost:8765`
- Run `rviz2` in a new terminal and add the topics you want to visualize
