#include <string>

#include "../include/waypoint_publisher.hpp"

using std::placeholders::_1;

WaypointPublisher::WaypointPublisher() 
: Node("waypoint_publisher"), _gpsData("waypoint_publisher")
{
    // Default values should be overriden by launch file params
    this->declare_parameter("waypoints_file_path",  
        "src/nav_stack/gps_service_files/config/waypoints.txt");
    this->declare_parameter("facing_north", true);

    std::string waypointsFilePath = this->get_parameter("waypoints_file_path").as_string();
    bool facingNorth = this->get_parameter("facing_north").as_bool();

    RCLCPP_INFO(this->get_logger(), "waypointsFilePath: %s, facingNorth: %d", 
        waypointsFilePath.c_str(), facingNorth);

    _gpsData.initializeMapInfo(waypointsFilePath, facingNorth);

    _waypoint_subscriber = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "gps_coords", 10, std::bind(&WaypointPublisher::waypointSubscriber, this, _1));
}

void WaypointPublisher::waypointSubscriber(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Longitude: %f, latitude: %f\n", msg->longitude, msg->latitude);
    _gpsData.setRobotCurrentLocation(msg->latitude, msg->longitude);
}

