#include <iostream>

#include "../include/waypoint_publisher.hpp"

int main(int argc, char * argv[])
{
    std::cout << "Running waypoint publisher node\n";
    
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WaypointPublisher>());
	rclcpp::shutdown();
	return 0;
}