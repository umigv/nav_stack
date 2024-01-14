#!/bin/bash
set -e

git submodule update --init --recursive simulation/velodyne_simulator

sudo apt update
sudo apt install zstd ros-humble-rttest ros-humble-rclcpp-action ros-humble-gazebo-dev ros-humble-gazebo-msgs ros-humble-gazebo-plugins ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs ros-humble-joint-state-publisher-gui ros-humble-xacro

if ! [[ -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]]; then
    sudo rosdep init
fi

set +e
source /opt/ros/humble/setup.bash
set -e
rosdep update
rosdep install --from-paths simulation/zed-ros2-wrapper --ignore-src -r -y

NVIDIA_GPU=false 

#https://www.linuxscrew.com/bash-prompt-for-input
if ! (sudo lshw -C display | grep -q "NVIDIA"); then
    while true; do
    read -p "No NVIDIA GPU detected. Do you have an NVIDIA GPU? [Y/N]: " answer
    case $answer in
        [Yy]* ) 
            NVIDIA_GPU=true
            break
        ;;
        [Nn]* ) 
            NVIDIA_GPU=false
            break
        ;;
        * ) echo "Answer either 'Y' or 'N'!";;
    esac
done
else
    NVIDIA_GPU=true
fi

if [ "$NVIDIA_GPU" = true ]; then
    git submodule update --init --recursive simulation/zed-ros2-wrapper

    ZEDSDKFILENAME="zedsdkubuntu22.run"

    if ! [[ -f $ZEDSDKFILENAME ]]; then
        wget https://download.stereolabs.com/zedsdk/4.0/cu121/ubuntu22 -O $ZEDSDKFILENAME
    fi
    chmod +x $ZEDSDKFILENAME

    export USER=$(id -u -n)
    sudo mkdir -p /etc/udev/rules.d/
    sudo apt install udev
    sudo apt install python3-requests
    
    ./$ZEDSDKFILENAME
else
    # removes zed camera from model, which is only supported with nvidia gpus
    sed -i '153,164d' simulation/marvin_simulation/urdf/marvin.xacro
fi

source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

echo "Build successful! Please remove the zed sdk installer file if everything is working correctly!"
