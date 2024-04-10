# GPS Transform 
This node allows you to automatically set waypoints to the planner given gps, odometry, and costmap data

## Dependencies
### Packages
- ```nav_stack/global_planner```
- ```nav_stack/gps_coord_pub```

### Subscribed Topics
- ```/map```
- ```/map_metadata```
- ```/tf```

### Called Actions
- ```/navigate_to_pose```

## Setup
1. Install any dependencies for global_planner and gps_coord_pub
2. If testing is desired, also install simulation_stack
TODO: see if any extra packages would need to be installed, write scripts:

## Running the node
1. Have the robot lidar / odometry running either from simulation or with the physical robot
2. Start the GPS
3. Start the global planner
4. Start the GPS Transform node by calling ```ros2 launch gps_transform waypoints.launch.py [args]```

## Launch File Arguments
### waypoints.launch.py
- ```waypoint_file``` (default: ```waypoint.txt```) - The list of waypoints separated longitude latitude format
- ```goal_tolerance``` (default: ```2.0```) - How close the robot should be to a waypoint to switch to the next waypoint
- ```facing_north``` (default: ```True```) - Whether the robot starts facing north

## Testing
- Install and setup simulation stack
- TODO: have a test world
- TODO: have a sample gps file to run
