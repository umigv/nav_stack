
#include "../include/waypoint_publisher.hpp"

using std::placeholders::_1;

WaypointPublisher::WaypointPublisher() : Node("waypoint_publisher")
{
    _waypoint_subscriber = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "gps_coords", 10, std::bind(&WaypointPublisher::waypointSubscriber, this, _1));
}

void WaypointPublisher::waypointSubscriber(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Longitude: %f, latitude: %f\n", msg->longitude, msg->latitude);
}

