// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "gps_transform_classes.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/trigger.hpp"
// #include "nav_stack/srv/GoalRequestService.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

/*

class GPSTransform : public rclcpp::Node {
public:
    GPSData gpsData;

    GPSTransform() : Node("gps_transform") {

        gpsData = GPSData();
        publisher_ = this->create_publisher<std_msgs::msg::String>("gps_goals", 10);
        timer_ = this->create_wall_timer(1000ms, std::bind(&GPSTransform::timer_callback, this));
    }

private:
    void timer_callback() {
        
        auto message = std_msgs::msg::String();

        message.data = "Hi";

        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

*/


void service_callback(const std::shared_ptr<example_interfaces::srv::Trigger::Request> request, std::shared_ptr<example_interfaces::srv::Trigger::Response> response) {

    std::shared_ptr<example_interfaces::srv::Trigger::Request> a = request; // Ignore warning
    
    gpsData.setRobotCurrentLocation(1, 1);
    response->success = true;
    response->message = "Hi";
}


int main(int argc, char *argv[]) {
// https://get-help.robotigniteacademy.com/t/ros2-service-client-in-c-with-classes-with-node-inheritance/19647

    // GPSService gpsService = GPSService();
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<GPSTransform>());
    // rclcpp::shutdown();



    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("gps_transform_node");
    rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr service = node->create_service<example_interfaces::srv::Trigger>("get_gps_goal", &service_callback);

    rclcpp::spin(node);
    rclcpp::shutdown();

    
    GPSData gpsData;
    
    // Test GPS DATA
    if (DEBUG) {
        gpsData.printGPSData();
    }

    return 0;
}