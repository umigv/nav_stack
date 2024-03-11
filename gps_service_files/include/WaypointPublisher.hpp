#pragma once
#include <iostream>
#include <deque>
#include "Point.hpp"
#include "MapFrame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using std::placeholders::_1;

class WaypointPublisher : public rclcpp::Node{
    public:
    WaypointPublisher() : Node("WaypointPublisher"), tfBuffer(this->get_clock()), tfListener(tfBuffer){
        mapInfoSubscriber = this->create_subscription<nav_msgs::msg::MapMetaData>("mapInfo", 10, std::bind(&WaypointPublisher::mapInfoCallback, this, _1));
        robotGPSSubscriber = this->create_subscription<sensor_msgs::msg::NavSatFix>("gps_coords", 10, std::bind(&WaypointPublisher::robotGPSCallback, this, _1));
    }

    private:
    friend std::ostream& operator<<(std::ostream& os, const WaypointPublisher& waypointPublisher);

    void mapInfoCallback(const nav_msgs::msg::MapMetaData::SharedPtr msg){
        frame = MapFrame(Point(msg->origin.position.x, msg->origin.position.y), msg->width, msg->height, msg->resolution);
    }

    void robotGPSCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg){
        robotGPS = GPSCoordinate(msg->latitude, msg->longitude);
    }

    Point getRobotPosition() const{
        geometry_msgs::msg::TransformStamped transform;
        
        try{
            transform = tfBuffer.lookupTransform("map", "base_link", tf2::TimePointZero);
        }
        catch(tf2::TransformException& exception){
            RCLCPP_ERROR(this->get_logger(), "Could not get robot position: %s", exception.what());
            return Point();
        }

        return Point(transform.transform.translation.x, transform.transform.translation.y);
    }

    void update(){
        if(waypoints.empty()){
            return;
        }

        const Point robotPosition = getRobotPosition();
        const Point& waypoint = frame.constrainPoint(robotPosition + Point(robotGPS, waypoints.front()));

        if(!faceNorth){
            waypoint.rotateBy(M_PI);
        }

        if(robotPosition.distanceTo(waypoint) < kEpsilon){
            waypoints.pop_front();
            return;
        }

        // publish waypoint
    }

    std::deque<GPSCoordinate> waypoints;
    bool faceNorth;

    rclcpp::Subscription<nav_msgs::msg::MapMetaData>::SharedPtr mapInfoSubscriber;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr robotGPSSubscriber;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    MapFrame frame;
    GPSCoordinate robotGPS;
    long double kEpsilon{2.0};
};

std::ostream& operator<<(std::ostream& os, const WaypointPublisher& waypointPublisher){
    os << "Upcoming GPS Waypoints: ";
    for(const GPSCoordinate& waypoint : waypointPublisher.waypoints){
        os << waypoint << "\n";
    }
    return os;
}