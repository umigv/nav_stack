# nav_stack - Simulation
This repository includes a package that allow you to simulate Marvin (UMARV's 2022-2023 robot) using RViz and Gazebo

## urdf basics
Unified Robotics Description Format (urdf) is an XML specification used to model robots. It is made up of ```links``` (bodies with kinematic and dynamic specifications) and ```joints``` (connection between links). You can imagine the model structure to be a [tree](https://en.wikipedia.org/wiki/Tree_(data_structure)) with links as nodes and joints as edges. The root, or fixed frame (in our case ```chassis```, specified in ```simulation_config.rviz```) is the core of the model. All other links are define relative to the fixed frame. You can read more about the urdf format [here](https://wiki.ros.org/urdf/XML) and [here](https://navigation.ros.org/setup_guides/urdf/setup_urdf.html#urdf-and-the-robot-state-publisher).

## Dependencies
- ROS Humble, installed either through [UMARV's environment repository](https://github.com/umigv/environment) or [directly from source](https://docs.ros.org/en/humble/Installation.html) for linux
- ROS joint state publisher and xacro package. To install, run ```sudo apt install ros-humble-joint-state-publisher-gui``` and ```sudo apt install ros-humble-xacro```
- [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper), which simulates our camera. Follow the instructions [here](https://www.stereolabs.com/developers/release/) to install the ZED Camera SDK, [here](https://developer.nvidia.com/cuda-downloads) to install CUDA (NVidia GPU required), and [here](https://github.com/stereolabs/zed-ros2-wrapper#build-the-package) (replace ```ros2_ws``` with ```nav_stack```) to install the package
- [Velodyne Simulator](https://github.com/ToyotaResearchInstitute/velodyne_simulator), which simulates our LiDAR. To insall, cd into ```nav_stack/src```, then run ```git clone https://github.com/ToyotaResearchInstitute/velodyne_simulator.git```

## How to use
1. Clone this repo
2. cd into the src folder of the repository
3. Add packages ```zed-ros2-wrapper``` and ```velodyne_simulation``` into the src repository as illustrated in dependencies
4. Return to the top of the repository with ```cd ..```
5. Run the following commands:
    ```
    colcon build
    install/setup.bash
    ros2 launch marvin_simulation display.launch.py
    ```
A rviz window should open up with the robot, along with controllable wheels

## Sample Image
![image](https://github.com/umigv/nav_stack/assets/71594512/2207361a-ef23-4be5-b0e8-5814065190f8)


## Possible Issues
### ros2: command not found
Run ```source /opt/ros/humble/setup.bash```

### symbol lookup error: ... undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
This error is caused by the VSCode snap package setting GTK_PATH, which is then used by the VSCode integrated terminal. There are two solutions:
- run ``` unset GTK_PATH ```
- Build and run the project using an external terminal

### GTK+ module libcanberra-gtk-module.so cannot be loaded  
This error is caused by [snap variables leaking into terminal variables](https://github.com/microsoft/vscode/issues/179086). There are two solutions:
- (If linux) Redownload VSCode from the debian package [here](https://code.visualstudio.com/download)
- Build and run the project using an external terminal
