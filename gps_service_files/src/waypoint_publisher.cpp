#include <chrono>
#include <memory>

#include "../include/waypoint_publisher.hpp"

using std::placeholders::_1;

WaypointPublisher::WaypointPublisher()
: Node("waypoint_publisher")
{
    _waypoint_subscriber = this->create_subscription<sensor_msgs::msg::NavSatFix>(
		"fix", 10, std::bind(&WaypointPublisher::waypointSubscriber, this, _1));
}

void WaypointPublisher::waypointSubscriber(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), 
        "Longitude: %f, latitude: %f\n", msg->longitude, msg->latitude);
}




//    GPSData gpsData;
    
//     // Test GPS DATA
//     if (DEBUG) {
//         gpsData.printGPSData();
//     }