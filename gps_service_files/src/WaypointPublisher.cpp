#include <fstream>
#include "../include/WaypointPublisher.hpp"

using std::placeholders::_1;

WaypointPublisher::WaypointPublisher() : Node("WaypointPublisher"), tfBuffer(this->get_clock()), tfListener(tfBuffer){
    this->declare_parameter("waypoints_file", "waypoints.txt");
    this->declare_parameter("face_north", true);
    this->declare_parameter("change_waypoint_distance", 2.0);

    faceNorth = this->get_parameter("face_north").as_bool();
    kEpsilon = this->get_parameter("change_waypoint_distance").as_double();
    readWaypoints(this->get_parameter("waypoints_file").as_string());

    mapInfoSubscriber = this->create_subscription<nav_msgs::msg::MapMetaData>("mapInfo", 10, std::bind(&WaypointPublisher::mapInfoCallback, this, _1));
    robotGPSSubscriber = this->create_subscription<sensor_msgs::msg::NavSatFix>("gps_coords", 10, std::bind(&WaypointPublisher::robotGPSCallback, this, _1));
    waypointPublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("waypoint", 10);
    waypointUpdater = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&WaypointPublisher::updateWaypoint, this));
}

void WaypointPublisher::readWaypoints(const std::string& file){
    std::ifstream is(file.c_str());

    if(!is.is_open()){
        RCLCPP_ERROR(this->get_logger(), "Could not open waypoints file");
        return;
    }

    GPSCoordinate waypoint;
    while(is >> waypoint){
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
    frame = MapFrame(Point(map->origin.position.x, map->origin.position.y), map->width, map->height, map->resolution);
}

void WaypointPublisher::robotGPSCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gpsCoordinate){
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

void WaypointPublisher::updateWaypoint(){
    if(waypoints.empty()){
        return;
    }

    const Point robotPosition = getRobotPosition();
    const Point waypoint = frame.constrainPoint(robotPosition + Point(robotGPS, waypoints.front()));

    if(!faceNorth){
        waypoint.rotateBy(M_PI);
    }

    if(robotPosition.distanceTo(waypoint) < kEpsilon){
        waypoints.pop_front();
        return;
    }

    waypointPublisher->publish(waypoint.toPoseStamped());
}

std::ostream& operator<<(std::ostream& os, const WaypointPublisher& waypointPublisher){
    os << "Upcoming GPS Waypoints: ";
    for(const GPSCoordinate& waypoint : waypointPublisher.waypoints){
        os << waypoint << '\n';
    }
    return os;
}