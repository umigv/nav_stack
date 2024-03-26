#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>  // Fix: Corrected the include statement
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include "tf2_ros/buffer.h"
#include <vector>


using std::string;
using std::placeholders::_1;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using TransformStamped = geometry_msgs::msg::TransformStamped;  // Fix: Added semicolon

class MergeService : public rclcpp::Node{
    private: 

    rclcpp::Subscription<OccupancyGrid>::SharedPtr cv_og_subscriber;
    rclcpp::Subscription<OccupancyGrid>::SharedPtr sensors_occupancy_grid_subscriber;

    OccupancyGrid cv_occupancy_grid;
    OccupancyGrid sensors_occupancy_grid;

    // Declare and acquire `target_frame` parameter
    string target_frame_ = this->declare_parameter<string>("target_frame", "odom");

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // create a robotPose index pair variable
    std::pair<double, double> robotPoseCV;
    std::pair<double, double> robotPoseSlam;
    double pose_x, pose_y;

    template <typename T>
    // T must support being -1, 0, or 1.
    int8_t merge_cell(T slam_value, T lane_value){
        /*
        9 cases:
            LL = -1, SLAM = -1 -> -1
            LL = -1, SLAM = 0 -> 0
            LL = -1, SLAM = 1 -> 1
            LL = 0, SLAM = -1 -> 0
            LL = 0, SLAM = 0 -> 0
            LL = 0, SLAM = 1 -> 1 OLD: 0
            LL = 1, SLAM = -1 -> 1
            LL = 1, SLAM = 0 -> 1 OLD: 0
            LL = 1, SLAM = 1 -> 1
        */
        int8_t return_val = 0;
        if (lane_value == -1){
            if (slam_value == -1){
                return_val = -1;
            }
            else if (slam_value == 1){
                return_val = 1;   
            }
        }
        if (lane_value == 0){
            if (slam_value == 1){
                return_val = 1;
            }
        }
        if (lane_value == 1){
            return_val = 1;
        }
        return return_val;
    }

    public:
    MergeService();

    void occupancyGridSubscriber();

    OccupancyGrid mergeSLAMAndLaneLine(const OccupancyGrid &cv_cm, const OccupancyGrid &slam_cm);

    void printOccupancyGridInfo(const OccupancyGrid &grid);

    void populateSensorsOccupancyGrid(const OccupancyGrid::SharedPtr snsr_cm);

    void populateCvOccupancyGrid(const OccupancyGrid::SharedPtr cv_cm);

    void get_pose(double &pose_x, double &pose_y);

};