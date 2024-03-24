#include <fstream>
#include <string>
#include "../include/WaypointPublisher.hpp"


using std::placeholders::_1;
using std::placeholders::_2;
using nav2_msgs::action::NavigateToPose;
using NavigateToPoseGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

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
    robotGPSSubscriber = this->create_subscription<sensor_msgs::msg::NavSatFix>("gps/data", 10, std::bind(&WaypointPublisher::robotGPSCallback, this, _1));
    goalPoseClient = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    //goalPosePublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
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

    //RCLCPP_INFO(this->get_logger(), "origin: (%f, %f), width: %u, height: %u, resolution: %f", xpos, ypos, width, height, resolution);
        
    frame = MapFrame(Point(xpos, ypos), width, height, resolution);
}

void WaypointPublisher::robotGPSCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gpsCoordinate){
    //RCLCPP_INFO(this->get_logger(), "lat: %f, lon: %f", gpsCoordinate->latitude, gpsCoordinate->longitude);

    robotGPS = GPSCoordinate(gpsCoordinate->latitude, gpsCoordinate->longitude);
}

Point WaypointPublisher::getRobotPosition() const{
    geometry_msgs::msg::TransformStamped transform;
    
    try{
        transform = tfBuffer.lookupTransform("base_link", "map", rclcpp::Time(0));

    }
    catch(tf2::TransformException& exception){
        RCLCPP_ERROR(this->get_logger(), "Could not get robot position: %s", exception.what());
        return Point();
    }

    //RCLCPP_INFO(this->get_logger(), "get robot position success");
    return Point(transform.transform.translation.x, transform.transform.translation.y);
}

void WaypointPublisher::updateGoalPose(){
    if(waypoints.empty()){
        RCLCPP_INFO(this->get_logger(), "Out of waypoints");
        return;
    }

    const Point robotPosition = getRobotPosition();
    Point unconstrainedGoal = robotPosition + Point(robotGPS, waypoints.front());

    if(!faceNorth) {
        unconstrainedGoal = unconstrainedGoal.rotateBy(M_PI);
    }

    RCLCPP_INFO(this->get_logger(), "yeah the waypoint is cooking");

    if(robotPosition.distanceTo(unconstrainedGoal) < kEpsilon){
        RCLCPP_INFO(this->get_logger(), "yooo next point time");
        waypoints.pop_front();
        return;
    }

    const Point constrainedGoal = frame.constrainToMap(unconstrainedGoal);

    RCLCPP_INFO(this->get_logger(), "unconstrained goal: %Lf, %Lf", unconstrainedGoal.getX(), unconstrainedGoal.getY());
    RCLCPP_INFO(this->get_logger(), "constrained goal: %Lf, %Lf", constrainedGoal.getX(), constrainedGoal.getY());

    auto goal_pose_client_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    goal_pose_client_options.goal_response_callback = std::bind(&WaypointPublisher::goalResponseCallBack, this, _1);
    goal_pose_client_options.feedback_callback = std::bind(&WaypointPublisher::feedbackCallback, this, _1, _2);
    goal_pose_client_options.result_callback = std::bind(&WaypointPublisher::resultCallback, this, _1);
    
    goalPoseClient->async_send_goal(constrainedGoal.toNavigateToPoseGoal(), goal_pose_client_options);


    //goalPosePublisher->publish(constrainedGoal.toPoseStamped());
}

void WaypointPublisher::goalResponseCallBack(NavigateToPoseGoalHandle::SharedPtr future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void WaypointPublisher::feedbackCallback(NavigateToPoseGoalHandle::SharedPtr, 
                                           const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    // std::stringstream ss;
    // ss << "Next number in sequence received: ";
    // for (auto number : feedback->partial_sequence) {
    //   ss << number << " ";
    // }
    RCLCPP_INFO(this->get_logger(), "NavigateToPose feedback current pose: (%lf, %lf, %lf)", 
        feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y, feedback->current_pose.pose.position.z);
  }

  void WaypointPublisher::resultCallback(const NavigateToPoseGoalHandle::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal was successful");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
  }

std::ostream& operator<<(std::ostream& os, const WaypointPublisher& waypointPublisher){
    os << "Upcoming GPS Waypoints: ";
    for(const GPSCoordinate& waypoint : waypointPublisher.waypoints){
        os << waypoint << '\n';
    }

    return os;
}