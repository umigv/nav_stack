#pragma once
#include <iostream>
#include <deque>
#include "Point.hpp"
#include "MapFrame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class WaypointPublisher : public rclcpp::Node{
public:
    WaypointPublisher();

private:
    friend std::ostream& operator<<(std::ostream& os, const WaypointPublisher& waypointPublisher);

    void readWaypoints(std::istream& is);

    void mapInfoCallback(const nav_msgs::msg::MapMetaData::SharedPtr msg);

    void robotGPSCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    Point getRobotPosition() const;

    void updateGoalPose();

    // Map Subscriber
    rclcpp::Subscription<nav_msgs::msg::MapMetaData>::SharedPtr mapInfoSubscriber;
    MapFrame frame;

    // GPS Subscriber
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr robotGPSSubscriber;
    GPSCoordinate robotGPS;

    // TF2
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    // Publisher
    rclcpp::TimerBase::SharedPtr goalPoseUpdater;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goalPosePublisher;

    // Data
    std::deque<GPSCoordinate> waypoints;
    long double kEpsilon{2.0};
    bool faceNorth;
};

std::ostream& operator<<(std::ostream& os, const WaypointPublisher& waypointPublisher);