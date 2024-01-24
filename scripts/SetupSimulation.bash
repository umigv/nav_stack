#!/bin/bash
set -e

git submodule update --init --recursive simulation/velodyne_simulator
git submodule update --init --recursive simulation/pointcloud_to_laserscan

sudo apt update
sudo apt-get install ros-humble-rttest ros-humble-rclcpp-action ros-humble-gazebo-dev ros-humble-gazebo-msgs ros-humble-gazebo-plugins ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs ros-humble-joint-state-publisher-gui ros-humble-xacro

#https://www.linuxscrew.com/bash-prompt-for-input
while true; do
    read -p "Do you have an NVIDIA GPU? [Y/N]: " answer
    case $answer in
        [Yy]* ) 
            git submodule update --init --recursive simulation/zed-ros2-wrapper
            
            sudo apt-get install zstd python3-requests python3-rosdep wget
            
            if ! [[ -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]]; then
                sudo rosdep init
            fi
            
            rosdep update
            rosdep install --from-paths simulation/zed-ros2-wrapper --ignore-src -r -y

            ZEDSDKFILENAME="zedsdk.run"
        
            if ! [[ -f $ZEDSDKFILENAME ]]; then
                wget https://download.stereolabs.com/zedsdk/4.0/cu121/ubuntu22 -O $ZEDSDKFILENAME
            fi
            chmod +x $ZEDSDKFILENAME
        
            export USER=$(id -u -n)
            sudo mkdir -p /etc/udev/rules.d/
            
            ./$ZEDSDKFILENAME
            
            break
        ;;
        [Nn]* )
            # removes zed camera from model, which is only supported with nvidia gpus
            sed -i.with_zed -zE "s/    <\!-- ZED Camera -->(.|\n)*<\!-- Sim Camera/    <\!-- Sim Camera/" simulation/marvin_simulation/urdf/marvin.xacro
            
            break
        ;;
        * ) echo "Answer either 'Y' or 'N'!";;
    esac
done

cd ../../

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

echo "Build successful! Please remove the zed sdk installer file if everything is working correctly!"
