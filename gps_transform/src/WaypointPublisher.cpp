#include <fstream>
#include <string>
#include "../include/WaypointPublisher.hpp"

using std::placeholders::_1;

WaypointPublisher::WaypointPublisher() : Node("WaypointPublisher"), tfBuffer(this->get_clock()), tfListener(tfBuffer){
    this->declare_parameter("face_north", true);
    this->declare_parameter("goal_tolerance", 2.0);
    this->declare_parameter("waypoints_file", "waypoints.txt");

    faceNorth = this->get_parameter("face_north").as_bool();
    kEpsilon = this->get_parameter("goal_tolerance").as_double();
    const std::string waypoints_file_path = this->get_parameter("waypoints_file").as_string();
    std::ifstream is(waypoints_file_path);
    RCLCPP_INFO(this->get_logger(), waypoints_file_path.c_str());
    readWaypoints(is);

    mapInfoSubscriber = this->create_subscription<nav_msgs::msg::MapMetaData>("map_metadata", 10, std::bind(&WaypointPublisher::mapInfoCallback, this, _1));
    robotGPSSubscriber = this->create_subscription<sensor_msgs::msg::NavSatFix>("gps_coords", 10, std::bind(&WaypointPublisher::robotGPSCallback, this, _1));
    goalPosePublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pub", 10);
    goalPoseUpdater = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&WaypointPublisher::updateGoalPose, this));
}

void WaypointPublisher::readWaypoints(std::istream& is){
    if(is.fail()){
        RCLCPP_ERROR(this->get_logger(), "Could not open waypoints file");
        return;
    }

    GPSCoordinate waypoint;
    while(is >> waypoint){
        RCLCPP_INFO(this->get_logger(), 
            (std::to_string(waypoint.getLatitude()) + ", " + std::to_string(waypoint.getLongitude())).c_str());
        waypoints.emplace_back(waypoint);
    }

    if(waypoints.empty()){
        RCLCPP_ERROR(this->get_logger(), "No waypoints found in file");
        return;
    }

    waypoints.push_back(waypoints.front());

    if(!faceNorth){
        std::reverse(waypoints.begin(), waypoints.end());
    }
}

void WaypointPublisher::mapInfoCallback(const nav_msgs::msg::MapMetaData::SharedPtr map){
    double xpos = map->origin.position.x;
    double ypos = map->origin.position.y;
    uint32_t width = map->width; 
    uint32_t height = map->height;
    float resolution = map->resolution;
    
    RCLCPP_INFO(this->get_logger(), 
        "origin: (%f, %f), width: %u, height: %u, resolution: %f", 
        xpos, ypos, width, height, resolution);
        
    frame = MapFrame(Point(xpos, ypos), width, height, resolution);
}

void WaypointPublisher::robotGPSCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gpsCoordinate){
    RCLCPP_INFO(this->get_logger(), 
        "lat: %f, lon: %f", gpsCoordinate->latitude, gpsCoordinate->longitude);

    robotGPS = GPSCoordinate(gpsCoordinate->latitude, gpsCoordinate->longitude);
}

Point WaypointPublisher::getRobotPosition() const{
    geometry_msgs::msg::TransformStamped transform;
    
    try{
        transform = tfBuffer.lookupTransform("base_link", "map", tf2::TimePointZero);
    }
    catch(tf2::TransformException& exception){
        RCLCPP_ERROR(this->get_logger(), "Could not get robot position: %s", exception.what());
        return Point();
    }

    return Point(transform.transform.translation.x, transform.transform.translation.y);
}

void WaypointPublisher::updateGoalPose(){
    if(waypoints.empty()){
        return;
    }

    const Point robotPosition = getRobotPosition();
    const Point goal = frame.constrainToMap(robotPosition + Point(robotGPS, waypoints.front()));

    if(!faceNorth){
        goal.rotateBy(M_PI);
    }

    if(robotPosition.distanceTo(goal) < kEpsilon){
        waypoints.pop_front();
        return;
    }

    goalPosePublisher->publish(goal.toPoseStamped());
}

std::ostream& operator<<(std::ostream& os, const WaypointPublisher& waypointPublisher){
    os << "Upcoming GPS Waypoints: ";
    for(const GPSCoordinate& waypoint : waypointPublisher.waypoints){
        os << waypoint << '\n';
    }

    return os;
}