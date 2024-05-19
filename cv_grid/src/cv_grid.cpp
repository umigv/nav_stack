// Header file
#include "cv_grid.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include <vector>

using namespace std::chrono_literals;

cv_grid::cv_grid() : Node("cv_grid"), grid_x_(0), grid_y_(0) {
    // publish piece of global lane lines occupancy grid
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("cv_grid_out", 10);
    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&cv_grid::publishGrid, this));

    // subscribe to local lane lines occupancy grid
    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "occupancy_grid", 10, std::bind(&cv_grid::cv_grid_callback, this, std::placeholders::_1));

    // Declare and acquire `target_frame` parameter
    target_frame_ = this->declare_parameter<std::string>("target_frame", "odom");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);       
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Initialize sliding window lane lines grids
    window_height_ = 200;
    window_width_ = 200;
    curr_sliding_grid_ = std::vector<std::vector<int>>(window_height_, std::vector<int>(window_width_, -1));
    prev_sliding_grid_ = std::vector<std::vector<int>>(window_height_, std::vector<int>(window_width_, -1));
    first_lookup_ = true;

    // Need to publish an incorrect transform for initialization.
    window_grid_transform_publisher();
}



void cv_grid::publishGrid() {
    // geometry_msgs::msg::TransformStamped rob_pose = get_node_options.lookupTransform('baselink', 'odom', rclcpp::Time(0));

    std::vector<int8_t> gridData(window_width_ * window_height_, -1);  // initialize with unknown
    double rob_x;
    double rob_y;
    geometry_msgs::msg::Quaternion quat;
    get_pose(rob_x, rob_y, quat);

    for (int i = 0; i < window_width_ * window_height_; ++i) {
        gridData[i] = curr_sliding_grid_[i / window_width_][i % window_width_];
        if (gridData[i] == 2) {
            std::cout << "found robot pose." << std::endl;
            grid_x_ = i % window_width_;
            grid_y_ = i / window_width_;
        }
    }

    window_grid_transform_publisher();

    // Create the OccupancyGrid message
    auto occupancyGridMsg = nav_msgs::msg::OccupancyGrid();
    occupancyGridMsg.header.stamp = now();
    occupancyGridMsg.header.frame_id = "odom";
    occupancyGridMsg.info.width = window_width_;
    occupancyGridMsg.info.height = window_height_;
    occupancyGridMsg.info.resolution = RESOLUTION;  // Replace with your desired RESOLUTION
    // occupancyGridMsg.info.origin.position.x = rob_x - grid_x_ * RESOLUTION;
    // occupancyGridMsg.info.origin.position.y = rob_y - grid_y_ * RESOLUTION;
    // occupancyGridMsg.info.origin.position.z = 0.0;
    occupancyGridMsg.info.origin = get_cv_grid_origin().pose;

    // Convert the 1D grid data to 2D
    occupancyGridMsg.data = gridData;

    RCLCPP_INFO(this->get_logger(), "Publishing cv_grid");
    // Publish the OccupancyGrid message
    publisher_->publish(occupancyGridMsg);
}


void cv_grid::cv_grid_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr occ_grid) {   
    RCLCPP_INFO(this->get_logger(), "Entering cv_grid callback");

    geometry_msgs::msg::Quaternion quat;
    if (first_lookup_) {
        get_pose(prev_pose_x_, prev_pose_y_, quat);
        first_lookup_ = false;
    }

    curr_sliding_grid_ = std::vector<std::vector<int>>(window_height_, std::vector<int>(window_width_, -1));        
    

    double curr_pose_x, curr_pose_y;
    get_pose(curr_pose_x, curr_pose_y, quat);

    // Accounts for the adjusted shift as the robot moves forward, backward, left, right, etc.
    // This allows the cv_grid "window" to always contain data around the robot position.
    int trans_cols = (curr_pose_x - prev_pose_x_) / RESOLUTION;
    int trans_rows = (curr_pose_y - prev_pose_y_) / RESOLUTION;
    for (int i = 0; i < window_height_; i++) {
        for (int j = 0; j < window_width_; j++) {

            int transformed_i = i - trans_rows;
            int transformed_j = j - trans_cols;

            if ((transformed_i >= 0) && (transformed_j >= 0) && (transformed_i < window_height_) 
                    && (transformed_j < window_width_)) {
                    if (prev_sliding_grid_[i][j] == 2) {
                        prev_sliding_grid_[i][j] = 0;
                    }
                curr_sliding_grid_[transformed_i][transformed_j] = prev_sliding_grid_[i][j];
            }
        }
    }

    int cv_width = occ_grid->info.width;
    int cv_height = occ_grid->info.height;


    // Need to find where the 2 is in the occ_grid.
    for (int i = 0; i < cv_height; i++) {
        for (int j = 0; j < cv_width; j++) {
            if (occ_grid->data[(cv_height - i - 1)*cv_width + (cv_width - j - 1)] == 2) {
                grid_x_ = cv_width - j - 1;
                grid_y_ = cv_height - i - 1;
                break;                
            } 
        }
        if (grid_x_ != -1) {
            break;
        }
    }
    window_grid_transform_publisher();
    cv_view_transform_publisher();

    int robot_row = window_height_/2;
    int robot_col = window_width_/2; 


    for (int i = 0; i < cv_height; i++) {
        for (int j = 0; j < cv_width; j++) {
            geometry_msgs::msg::Point camera_view;
            camera_view.y = i;
            camera_view.x = j;
            camera_view.z = 0;
            // geometry_msgs::msg::Point cv_grid_location = cv_view_to_cv_grid(camera_view);
            int val = occ_grid->data[(cv_height - i - 1)*cv_width + (cv_width - j - 1)];
            if (val == 1) {
                val = 100;
            }
            else if (val == 0) {
                val = 0;
            }

            if (val >= 0) {
                curr_sliding_grid_[robot_row + i][robot_col - cv_width/2 + j] = val;
                // curr_sliding_grid_[cv_grid_location.y][cv_grid_location.x] = val;
            }
        }
        // std::cout << std::endl;
    }

    prev_sliding_grid_ = curr_sliding_grid_;
    prev_pose_x_ =  curr_pose_x;
    prev_pose_y_ =  curr_pose_y;
}

void cv_grid::window_grid_transform_publisher() {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "cv_grid";
    double rob_x;
    double rob_y;
    geometry_msgs::msg::Quaternion quat;
    get_pose(rob_x, rob_y, quat);

    // The current robot x coordinate in odom - the robot x coordinate in the grid * the grid RESOLUTION.
    transform.transform.translation.x = rob_x - grid_x_ * RESOLUTION;
    transform.transform.translation.y = rob_y - grid_y_ * RESOLUTION;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.w = 1.0;

    transform.header.stamp = this->now();
    broadcaster_->sendTransform(transform);
    RCLCPP_INFO(this->get_logger(), "Cv Grid Transform published");
}

void cv_grid::cv_view_transform_publisher() {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "base_footprint";
    transform.child_frame_id = "computer_vision_view";
    // The current robot x coordinate in odom - the robot x coordinate in the grid * the grid RESOLUTION.
    // transform.transform.translation.x = grid_x_in * RESOLUTION;
    // transform.transform.translation.y = grid_y_in * RESOLUTION;
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.w = 1.0;

    transform.header.stamp = this->now();
    broadcaster_->sendTransform(transform);
    RCLCPP_INFO(this->get_logger(), "Computer Vision View Transform published");
}

geometry_msgs::msg::Point cv_grid::cv_view_to_cv_grid(const geometry_msgs::msg::Point& coord) {
    geometry_msgs::msg::Point transformed_point;
    try
    {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("cv_grid", "base_link", rclcpp::Time(0));
        tf2::doTransform(coord, transformed_point, transform);
        // transformed_point.header.frame_id = "cv_grid";
        // transformed_point.header.stamp = this->now();
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to transform point: %s", ex.what());
        // Handle the exception appropriately
    }
    return transformed_point;
}

geometry_msgs::msg::PoseStamped cv_grid::get_cv_grid_origin() {
    geometry_msgs::msg::PoseStamped transformed_point;
    geometry_msgs::msg::PoseStamped input_point;
    input_point.pose.position.x = 0;
    input_point.pose.position.y = 0;
    input_point.pose.position.z = 0;
    input_point.pose.orientation.x = 0;
    input_point.pose.orientation.y = 0;
    input_point.pose.orientation.z = 0;
    input_point.pose.orientation.w = 1.0;

    try {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("odom", "cv_grid", rclcpp::Time(0));
        tf2::doTransform(input_point, transformed_point, transform);
        transformed_point.header.frame_id = "odom";
        transformed_point.header.stamp = this->now();
    }
    catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Failed to transform point: %s", ex.what());
        // Handle the exception appropriately
    }
    return transformed_point;
}

void cv_grid::get_pose(double &pose_x, double &pose_y, geometry_msgs::msg::Quaternion& quat) {
    std::string fromFrameRel = target_frame_.c_str();
    std::string toFrameRel = "base_link";
    geometry_msgs::msg::TransformStamped t;

    // Look up for the transformation between target_frame and turtle2 frames
    // and send velocity commands for turtle2 to reach target_frame
    try {
        t = tf_buffer_->lookupTransform(
        fromFrameRel,
        toFrameRel,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s",
        toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        return;
    }

    pose_x = t.transform.translation.x;
    pose_y = t.transform.translation.y;
    quat = t.transform.rotation;
    
    return;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cv_grid>());
    rclcpp::shutdown();
    return 0;
}
