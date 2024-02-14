#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gps_data.hpp"

using namespace std::chrono_literals;


class WaypointPublisher : public rclcpp::Node
{
public:
    WaypointPublisher();

private:
    void waypointSubscriber(const sensor_msgs::msg::NavSatFix::SharedPtr msg);


    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _waypoint_subscriber;
    GPSData _gpsData;
};