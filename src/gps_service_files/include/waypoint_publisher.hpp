#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "coordinate.hpp"
#include "gps_data.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class WaypointPublisher : public rclcpp::Node
{
public:
  WaypointPublisher();

private:
  void waypointSubscriber(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _waypoint_subscriber;
};