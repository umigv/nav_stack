FROM ros:humble

ENV ROS_HOME=/opt/ros_home

WORKDIR /tmp/src/nav_stack
COPY . .

RUN apt-get update \
 && apt-get install -y python3-pip \
 && rosdep update \
 && rosdep install --from-paths /tmp/src/nav_stack --ignore-src -r -y \
 && rm -rf /tmp/src

WORKDIR /
