#pragma once
#include "Point.hpp"
#include "rclcpp/rclcpp.hpp"

class Map{
    public:
    Map(const Point& origin, long double width, long double height, long double resolution){}

    Point constrainPoint(const Point& point);

    private:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _waypoint_subscriber;

    long double width;
    long double height;
};
