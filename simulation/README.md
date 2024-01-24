# Navigation Stack - Simulation
This package allows you to simulate Marvin (UMARV's 2022-2023 robot) using RViz and Gazebo

## URDF basics
Unified Robotics Description Format (URDF) is an XML specification used to model robots. It is made up of ```links``` (bodies with kinematic and dynamic specifications) and ```joints``` (connection between links). You can imagine the model structure to be a [tree](https://en.wikipedia.org/wiki/Tree_(data_structure)) with links as nodes and joints as edges. The root, or fixed frame (in our case ```chassis```, specified in ```simulation_config.rviz```) is the core of the model. All other links are define relative to the fixed frame. You can read more about the URDF format [here](https://wiki.ros.org/urdf/XML) and [here](https://navigation.ros.org/setup_guides/urdf/setup_urdf.html#urdf-and-the-robot-state-publisher). I also recommend watching [this video](https://youtu.be/CwdbsvcpOHM?si=mOkKDYqQnHFhNE2T) as it is a good introduction to URDF.

## Package Dependencies
- [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper), which simulates our camera.
- [Velodyne Simulator](https://github.com/ToyotaResearchInstitute/velodyne_simulator), which simulates our LiDAR.
- [pointcloud_to_laserscan](https://github.com/ros-perception/pointcloud_to_laserscan), which helps convert our LiDAR's point cloud data to laser scan which can then be used for SLAM.
- If you are not using the official UMARV environment, you should have ROS2 installed by following the tutorial [here](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)

## Requirements
### Nvidia GPU
In order to run the ZED camera within the simulation, an Nvidia GPU with CUDA capability is required. The installation guide below also includes steps to remove this dependency.
### Docker image setup (umarv environment)
**WARNING - this step recreates your docker container, which resets everything except files within the ```ws``` folder. Make sure your files are within ```ws``` or backed up.**
1. Open file explorer, find docker-compose.yml within your wsl folder
2. Open the file, uncomment line 16. This prevents errors when running Gazebo
3. Connect to the docker container as usual

## Quick Installation Guide (Recommended)

### Install Navigation Stack
1. ```cd``` into ```ws/src```
2. ```git clone -b simulation https://github.com/umigv/nav_stack.git```
3. ```cd nav_stack```

### Run the setup script
1. ```./scripts/SetupSimulation.bash```
2. Follow the prompt

## Manual Installation Guide 
**All commands should be run in ```ws/src/nav_stack``` unless otherwise specified**

### Install ROS2 Depedency 
Run the following command anywhere on your computer:  
```
sudo apt update && sudo apt install ros-humble-rttest \
ros-humble-rclcpp-action \
ros-humble-gazebo-dev \
ros-humble-gazebo-msgs \
ros-humble-gazebo-plugins \
ros-humble-gazebo-ros \
ros-humble-gazebo-ros-pkgs \
ros-humble-joint-state-publisher-gui \
ros-humble-xacro \
```

### Install the required packages
1. ```cd``` into ```ws/src```
2. ```git clone -b simulation https://github.com/umigv/nav_stack.git```
3. ```cd nav_stack```
4. ```git submodule update --init --recursive simulation/velodyne_simulator```
5. ```git submodule update --init --recursive simulation/pointcloud_to_laserscan```

### ZED Camera Installation (requires an Nvidia GPU)

#### Install the ZED Camera package
1. ```sudo apt-get install zstd python3-requests python3-rosdep wget```
2. ```git submodule update --init --recursive simulation/zed-ros2-wrapper```
3. ```sudo rosdep init```
4. ```rosdep update```
5. ```rosdep install --from-paths simulation/zed-ros2-wrapper --ignore-src -r -y```

#### Install CUDA and the ZED SDK
1. ```wget https://download.stereolabs.com/zedsdk/4.0/cu121/ubuntu22 -O zedsdk.run```
2. ```export USER=$(id -u -n)```
3. ```sudo mkdir -p /etc/udev/rules.d/```
4. ```sudo chmod +x zedsdk.run```
5. ```./zedsdk.run```
6. Press q, then follow the installation instructions
7. Once the ZED SDK is installed correctly, delete the installation file with ```rm -rf zedsdk.run```

### ZED Camera removal (if no Nvidia GPU)
1. ```sed -i.with_zed -zE "s/    <\!-- ZED Camera -->(.|\n)*<\!-- Sim Camera/    <\!-- Sim Camera/" simulation/marvin_simulation/urdf/marvin.xacro```

## Testing the Project
1. ```cd``` into ```ws```
2. ```source /opt/ros/humble/setup.bash```
3. ```colcon build --symlink-install```
4. ```source install/setup.bash```
5. ```ros2 launch marvin_simulation display.launch.py```, this should open up the robot model with controllable wheels with RViz```
6. ```ros2 launch marvin_simulation gazebo.launch.py```, this should open up the robot model with Gazebo```

## Sample Image
![image](https://github.com/umigv/nav_stack/assets/71594512/cde0a60f-b5a3-47b7-b05a-c7afba1f751d)
![image](https://github.com/umigv/nav_stack/assets/71594512/0ef3b50e-5b1a-42f2-a5a8-bbf3d5d2e234)

## Possible Issues
### ros2: command not found
Run ```source /opt/ros/humble/setup.bash```

### symbol lookup error: ... undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
This error is caused by the VSCode snap package setting GTK_PATH, which is then used by the VSCode integrated terminal. There are two solutions:
- run ``` unset GTK_PATH ```
- Build and run the project using an external terminal

### GTK+ module libcanberra-gtk-module.so cannot be loaded  
This error is caused by [snap variables leaking into terminal variables](https://github.com/microsoft/vscode/issues/179086). There are two solutions:
- (If linux) Redownload VSCode using the debian package [here](https://code.visualstudio.com/download)
- Build and run the project using an external terminal

## Credits
Jason Ning and Kari Naga on the sensors team, who created the original URDF files and the Gazebo World in the [marvin](https://github.com/umigv/marvin/tree/main/urdf) repository.  
Ethan Hardy for testing the package and creating the installation script
