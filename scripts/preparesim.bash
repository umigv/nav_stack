#!/bin/bash
set -e

git submodule update --init --recursive simulation/velodyne_simulator

sudo apt update
sudo apt-get install ros-humble-rttest ros-humble-rclcpp-action ros-humble-gazebo-dev ros-humble-gazebo-msgs ros-humble-gazebo-plugins ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs ros-humble-joint-state-publisher-gui ros-humble-xacro python3-rosdep

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
while true; do
    read -p "No NVIDIA GPU detected. Do you have an NVIDIA GPU? [Y/N]: " answer
    case $answer in
        [Yy]* ) 
            git submodule update --init --recursive simulation/zed-ros2-wrapper

            ZEDSDKFILENAME="zedsdk.run"
        
            if ! [[ -f $ZEDSDKFILENAME ]]; then
                wget https://download.stereolabs.com/zedsdk/4.0/cu121/ubuntu22 -O $ZEDSDKFILENAME
            fi
            chmod +x $ZEDSDKFILENAME
        
            export USER=$(id -u -n)
            sudo mkdir -p /etc/udev/rules.d/
            sudo apt-get install zstd python3-requests
            
            ./$ZEDSDKFILENAME
            
            break
        ;;
        [Nn]* )
            # removes zed camera from model, which is only supported with nvidia gpus
            sed -i '153,164d' simulation/marvin_simulation/urdf/marvin.xacro
            
            break
        ;;
        * ) echo "Answer either 'Y' or 'N'!";;
    esac
done

source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

echo "Build successful! Please remove the zed sdk installer file if everything is working correctly!"
