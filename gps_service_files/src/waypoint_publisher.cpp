#include <string>

#include "../include/waypoint_publisher.hpp"

using std::placeholders::_1;

WaypointPublisher::WaypointPublisher() 
: Node("waypoint_publisher"), _gpsData("waypoint_publisher")
{
    // Default values should be overrided by launch file params
    this->declare_parameter("waypoints_file_path",  
        "src/nav_stack/gps_service_files/config/waypoints.txt");
    this->declare_parameter("config_file_path", 
        "src/nav_stack/gps_service_files/config/config.yaml");

    std::string waypointsFilePath = this->get_parameter("waypoints_file_path").as_string();
    std::string configFilePath = this->get_parameter("config_file_path").as_string();

    RCLCPP_INFO(this->get_logger(), "waypointsFilePath: %s, configFilePath: %s", 
        waypointsFilePath.c_str(), configFilePath.c_str());

    _gpsData.initializeMapInfo(waypointsFilePath, configFilePath);

    _waypoint_subscriber = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "gps_coords", 10, std::bind(&WaypointPublisher::waypointSubscriber, this, _1));
}

void WaypointPublisher::waypointSubscriber(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Longitude: %f, latitude: %f\n", msg->longitude, msg->latitude);
    _gpsData.setRobotCurrentLocation(msg->latitude, msg->longitude);
}

