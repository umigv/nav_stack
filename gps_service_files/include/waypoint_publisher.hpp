#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class WaypointPublisher : public rclcpp::Node
{
public:
	WaypointPublisher()
	: Node("waypoint_publisher"), count_(0)
	{
        _waypoint_publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
        _timer = this->create_wall_timer(
            500ms, std::bind(&WaypointPublisher::timer_callback, this));
	}

private:
	void timer_callback()
	{
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
	}

	rclcpp::TimerBase::SharedPtr _timer;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _waypoint_publisher;
	size_t count_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WaypointPublisher>());
	rclcpp::shutdown();
	return 0;
}