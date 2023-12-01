# Navigation Stack - Simulation
This repository includes a package that allow you to simulate Marvin (UMARV's 2022-2023 robot) using RViz and Gazebo

## URDF basics
Unified Robotics Description Format (URDF) is an XML specification used to model robots. It is made up of ```links``` (bodies with kinematic and dynamic specifications) and ```joints``` (connection between links). You can imagine the model structure to be a [tree](https://en.wikipedia.org/wiki/Tree_(data_structure)) with links as nodes and joints as edges. The root, or fixed frame (in our case ```chassis```, specified in ```simulation_config.rviz```) is the core of the model. All other links are define relative to the fixed frame. You can read more about the URDF format [here](https://wiki.ros.org/urdf/XML) and [here](https://navigation.ros.org/setup_guides/urdf/setup_urdf.html#urdf-and-the-robot-state-publisher). I also recommend watching [this video](https://youtu.be/CwdbsvcpOHM?si=mOkKDYqQnHFhNE2T) as it is a good introduction to URDF.

## Package Dependencies
- [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper), which simulates our camera. Installation instructions below.
- [Velodyne Simulator](https://github.com/ToyotaResearchInstitute/velodyne_simulator), which simulates our LiDAR. Installation instructions below

## Installation Guide (umarv environment)
### Docker image setup
**WARNING - this step recreates your docker container, which resets everything except files within the ```ws``` folder. Make sure your files are within ```ws``` or backed up.**
1. Open file explorer, find docker-compose.yml within your wsl folder
2. Open the file, uncomment line 16 
3. Connect to the docker container as usual

### Install the navigation stack simulation package
1. ```cd``` into ```ws/src```
2. ```git clone -b simulation https://github.com/umigv/nav_stack.git```

### Install the velodyne LiDAR package
1. ```cd``` into ```ws/src/nav_stack/src/simulation```
2. ```git clone https://github.com/ToyotaResearchInstitute/velodyne_simulator.git```

### ZED Camera Installation (requires an Nvidia GPU)
#### Install the ZED Camera package
1. ```cd``` into ```ws/src/nav_stack/src/simulation```
2. ```git clone https://github.com/stereolabs/zed-ros2-wrapper.git```
3. ```cd``` into ```ws/src/nav_stack``` (go up two levels)
4. ```sudo apt update```
5. ```source /opt/ros/humble/setup.bash```
6. ```rosdep update```
7. ```rosdep install --from-paths src --ignore-src -r -y```

#### Install CUDA and the ZED SDK
1. Go to [this website](https://www.stereolabs.com/developers/release/), go to SDK Downloads and download CUDA 12 -> ZED SDK for ubuntu 22 to anywhere on your windows computer
2. Open a windows terminal and ```cd``` to where you downloaded the file
3. ```docker cp ZED_SDK_Ubuntu22_cuda12.1_v4.0.8.zstd.run umarv-ros2:home/umarv```
4. ```cd ~``` and ```ls```, you should see the file copied over
5. ```cd /```
6. ```export USER=umarv```
7. ```sudo mkdir -p /etc/udev/rules.d/```
8. ```cd ~```
9. ```sudo chmod +x ZED_SDK_Ubuntu22_cuda12.1_v4.0.8.zstd.run```
10. ```sudo apt install zstd```
11. ```./ZED_SDK_Ubuntu22_cuda12.1_v4.0.8.zstd.run```
12. Press q, then press y for everything (ZED diagnostics is optional)

### ZED Camera removal (if no Nvidia GPU)
1. Open marvin_new.xacro in nav_stack/src/simulation/marvin_simulation/urdf
2. Delete everything from line 153 to 164 (everything under the ZED Camera header). This removes ZED Camera as a dependency to our model


## Installation Guide (Linux)
### Install ROS2 Humble
Follow the tutorial [here](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)

### Install ROS2 Depedency 
Run the following command:
```
sudo apt install ros-humble-rttest \
ros-humble-rclcpp-action \
ros-humble-gazebo-dev \
ros-humble-gazebo-msgs \
ros-humble-gazebo-plugins \
ros-humble-gazebo-ros \
ros-humble-gazebo-ros-pkgs \
ros-humble-joint-state-publisher-gui \
ros-humble-xacro
```

### Install the navigation stack simulation and Velodyne LiDAR package
Same process as the umarv environment installation

### ZED Camera installation (requires an Nvidia GPU)
#### Install the ZED Camera package
Same process as the umarv environment installation

#### Install CUDA and the ZED SDK
1. Go to [this website](https://www.stereolabs.com/developers/release/), go to SDK Downloads and download CUDA 12 -> ZED SDK for ubuntu 22 to anywhere
2. ```cd``` to where you downloaded the file
9. ```sudo chmod +x ZED_SDK_Ubuntu22_cuda12.1_v4.0.8.zstd.run```
10. ```sudo apt install zstd```
11. ```./ZED_SDK_Ubuntu22_cuda12.1_v4.0.8.zstd.run```
12. Press q, then press y for everything (ZED diagnostics is optional)
    
### ZED Camera removal (if no Nvidia GPU)
Same process as the umarv environment installation

## Testing the Project
1. ```cd``` into ```ws/nav_stack```
2. ```source /opt/ros/humble/setup.bash```
3. ```colcon build```
4. ```. install/setup.bash```
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
The original URDF files and the Gazebo World are created by Jason Ning and Kari Naga on the sensors team, taken from the [marvin](https://github.com/umigv/marvin/tree/main/urdf) repository
