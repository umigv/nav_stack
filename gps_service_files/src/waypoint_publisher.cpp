#include <chrono>
#include <memory>

#include "../include/waypoint_publisher.hpp"


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WaypointPublisher>());
	rclcpp::shutdown();
	return 0;
}


//    GPSData gpsData;
    
//     // Test GPS DATA
//     if (DEBUG) {
//         gpsData.printGPSData();
//     }