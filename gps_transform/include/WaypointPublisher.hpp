#pragma once

#include <iostream>
#include <deque>

#include "Point.hpp"
#include "MapFrame.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateToPoseGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

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
     * @param map The map metadata message
    */
    void mapInfoCallback(const nav_msgs::msg::MapMetaData::SharedPtr map);

    /**
     * @brief Callback for the robot GPS subscriber
     * 
     * @param gpsCoordinate The robot GPS message
    */
    void robotGPSCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gpsCoordinate);

    void robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr robotOdom);

    /**
     * @brief Get the robot's position in the map frame
     * 
     * @return The robot's position in the map frame
    */
    Point getRobotPosition() const;

    /**
     * @brief Get the current unconstrained goal
     * 
     * @return The current unconstrained goal
    */
    Point getUnconstrainedGoal() const;

    /**
     * @brief If the robot has reached its current goal, updates the current goal to
     * the next waypoint in waypoints.txt
    */
    void updateCurrentGoal(const Point& currPosition);

    /**
     * @brief Update the goal pose based on the robot's GPS coordinates
    */
    void navigateToGoal();

    void goalResponseCallBack(NavigateToPoseGoalHandle::SharedPtr future);

    void feedbackCallback(NavigateToPoseGoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);

    void resultCallback(const NavigateToPoseGoalHandle::WrappedResult & result);

    // Map Subscriber
    rclcpp::Subscription<nav_msgs::msg::MapMetaData>::SharedPtr mapInfoSubscriber;
    MapFrame frame;

    // GPS Subscriber
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr robotGPSSubscriber;
    rclcpp::CallbackGroup::SharedPtr gpsCallbackGroup;
    GPSCoordinate robotGPS;

    // Odom subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robotPoseSubscriber;
    Point robotPose;

    // TF2
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    // Action
    rclcpp::CallbackGroup::SharedPtr navigateCallbackGroup;
    rclcpp::TimerBase::SharedPtr navigateToGoalTimer;
    rclcpp_action::Client<NavigateToPose>::SharedPtr goalPoseClient;

    // Goal update
    rclcpp::CallbackGroup::SharedPtr updateGoalCallbackGroup;
    rclcpp::TimerBase::SharedPtr updateGoalTimer;

    // Data
    std::deque<GPSCoordinate> waypoints;
    long double kEpsilon{2.0};
    bool faceNorth;
    bool mapInitialized = false;
    bool navigationInProgress = false;
};

/**
 * @brief prints out all remaining waypoints in the waypoint publisher
 * 
 * @param os The output stream
 * @param waypointPublisher The waypoint publisher to output
*/
std::ostream& operator<<(std::ostream& os, const WaypointPublisher& waypointPublisher);