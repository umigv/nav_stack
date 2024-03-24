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

/**
 * @brief A class to publish read GPS waypoints as goal poses.
 * 
 * The waypoints are read from a file and published as goal poses. The goal poses are updated based on the robot's GPS coordinates.
 * The specific math is explained in the wiki
 * 
 * Param:
 * - waypoints_file: The file containing the GPS waypoints
 * - face_north: Whether the robot starts off facing north
 * - epsilon: The distance in meters to consider the robot at a waypoint
 * 
 * Subscribed topics:
 * - /map_metadata (nav_msgs::msg::MapMetaData): The metadata of the occupancy grid
 * - /gps_data (sensor_msgs::msg::NavSatFix): The GPS coordinates of the robot
 * - /tf (tf2_msgs::msg::TFMessage): The transform between the map and base_link
*/
class WaypointPublisher : public rclcpp::Node{
public:
    WaypointPublisher();

private:
    friend std::ostream& operator<<(std::ostream& os, const WaypointPublisher& waypointPublisher);

    /**
     * @brief Read the GPS waypoints from a file
     * 
     * The first point of the file is appended to the back so that the robot drives back to the start
     * If the robot is not facing north, the waypoints are reversed so that the robot drives facing the correct direction
    */
    void readWaypoints(std::istream& is);

    /**
     * @brief Callback for the map metadata subscriber
     * 
     * @param msg The map metadata message
    */
    void mapInfoCallback(const nav_msgs::msg::MapMetaData::SharedPtr msg);

    /**
     * @brief Callback for the robot GPS subscriber
     * 
     * @param msg The robot GPS message
    */
    void robotGPSCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    /**
     * @brief Get the robot's position in the map frame
     * 
     * @return The robot's position in the map frame
    */
    Point getRobotPosition() const;

    /**
     * @brief Update the goal pose based on the robot's GPS coordinates
    */
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

/**
 * @brief prints out all remaining waypoints in the waypoint publisher
 * 
 * @param os The output stream
 * @param waypointPublisher The waypoint publisher to output
*/
std::ostream& operator<<(std::ostream& os, const WaypointPublisher& waypointPublisher);