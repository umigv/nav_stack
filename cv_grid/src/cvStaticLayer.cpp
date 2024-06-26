// Header file
#include "cvStaticLayer.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include <vector>
#include "pluginlib/class_list_macros.hpp"

using namespace std::chrono_literals;
namespace nav2_cv_static_layer {

// cvStaticLayer::cvStaticLayer() : 
//     Node("cvStaticLayer"), grid_x_(-1), grid_y_(-1), use_sim_time_(false),
//     window_height_(200), window_width_(200), resolution_(0.05) {
//     this->declare_parameter("Height", 200);
//     this->declare_parameter("Width", 200);
//     this->declare_parameter("Resolution", 0.05);
//     this->declare_parameter("debug", false);
//     std::string grid_topic = "";
//     this->declare_parameter("grid_topic", "");
//     std::string output_topic = "";
//     this->declare_parameter("output_topic", "");

//     this->get_parameter("output_topic", output_topic);
//     this->get_parameter("grid_topic", grid_topic);
//     this->get_parameter("use_sim_time", use_sim_time_);
//     this->get_parameter("Height", window_height_);
//     this->get_parameter("Width", window_width_);
//     this->get_parameter("Resolution", resolution_);
//     this->get_parameter("debug", debug_);

//     // publish piece of global lane lines occupancy grid
//     publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic, 10);
//     timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&cvStaticLayer::publishGrid, this));

//     // subscribe to local lane lines occupancy grid
//     subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
//         grid_topic, 10, std::bind(&cvStaticLayer::cv_grid_callback, this, std::placeholders::_1));

//     // Declare and acquire `target_frame` parameter
//     target_frame_ = this->declare_parameter<std::string>("target_frame", "odom");

//     tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
//     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);       
//     // broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
//     // cv_view_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

//     // Initialize sliding window lane lines grids
//     curr_sliding_grid_ = std::vector<std::vector<int>>(window_height_, std::vector<int>(window_width_, -1));
//     prev_sliding_grid_ = std::vector<std::vector<int>>(window_height_, std::vector<int>(window_width_, -1));
//     first_lookup_ = true;

//     RCLCPP_INFO(this->get_logger(), "Grid Size %li", curr_sliding_grid_.size());
//     RCLCPP_INFO(this->get_logger(), "Grid Size %li", prev_sliding_grid_.size());
// }

void cvStaticLayer::onInitialize() {
    node = node_.lock();
    node->declare_parameter("Height", 200);
    node->declare_parameter("Width", 200);
    node->declare_parameter("Resolution", 0.05);
    node->declare_parameter("debug", false);
    std::string grid_topic = "";
    node->declare_parameter("grid_topic", "");
    std::string output_topic = "";
    node->declare_parameter("output_topic", "");
    target_frame_ = node->declare_parameter<std::string>("target_frame", "odom");
    node->get_parameter("output_topic", output_topic);
    node->get_parameter("grid_topic", grid_topic);
    node->get_parameter("use_sim_time", use_sim_time_);
    node->get_parameter("Height", window_height_);
    node->get_parameter("Width", window_width_);
    node->get_parameter("Resolution", resolution_);
    node->get_parameter("debug", debug_);
    subscription_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        grid_topic, 10, std::bind(&cvStaticLayer::cv_grid_callback, this, std::placeholders::_1));
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);    
    curr_sliding_grid_ = std::vector<std::vector<int>>(window_height_, std::vector<int>(window_width_, -1));
    prev_sliding_grid_ = std::vector<std::vector<int>>(window_height_, std::vector<int>(window_width_, -1));
    first_lookup_ = true;
    RCLCPP_WARN(node->get_logger(), "Initializing cvStaticLayer!");
}

void cvStaticLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y) {
    if (!enabled_) {
        return;
    }
    // This is the upper left most point in the grid. Has highest y, lowest x.
    geometry_msgs::msg::PoseStamped origin = get_cv_grid_origin();
    *min_x = origin.pose.position.x;
    *max_x = origin.pose.position.x + window_width_;

    *min_y = origin.pose.position.y + window_height_;
    *max_y = origin.pose.position.y;
}

void cvStaticLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
    if (!enabled_) {
        return;
    }

    for (int i = min_i; i < max_i; ++i) {
        for (int j = min_j; j < max_j; ++j) {
            // int index = master_grid.getIndex(i, j);
            int value = curr_sliding_grid_[i][j];

            if (value == 0) {
                master_grid.setCost(i, j, nav2_costmap_2d::FREE_SPACE);
            }

            if (value == 100) {
                master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
            }
        }
    }
}

void cvStaticLayer::reset() {
    return;
}

bool isClearable() {
    return false;
}

// void cvStaticLayer::publishGrid() {
//     std::vector<int8_t> gridData(window_width_ * window_height_, -1);  // initialize with unknown

//     grid_lock_.lock();
//     for (int i = 0; i < window_width_ * window_height_; ++i) {
//         gridData[i] = curr_sliding_grid_[i / window_width_][i % window_width_];
//     }
//     grid_lock_.unlock();

//     // Create the OccupancyGrid message
//     auto occupancyGridMsg = nav_msgs::msg::OccupancyGrid();
//     occupancyGridMsg.header.stamp = this->get_clock()->now();
//     occupancyGridMsg.header.frame_id = "odom";
//     occupancyGridMsg.info.width = window_width_;
//     occupancyGridMsg.info.height = window_height_;
//     occupancyGridMsg.info.resolution = resolution_;  // Replace with your desired resolution_
//     occupancyGridMsg.info.origin = get_cv_grid_origin().pose;

//     // Convert the 1D grid data to 2D
//     occupancyGridMsg.data = gridData;

//     RCLCPP_INFO(this->get_logger(), "Publishing cvStaticLayer");
//     // Publish the OccupancyGrid message
//     publisher_->publish(occupancyGridMsg);
// }


void cvStaticLayer::cv_grid_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr occ_grid) {   
    RCLCPP_INFO(node->get_logger(), "Entering cvStaticLayer callback");

    geometry_msgs::msg::Quaternion quat;
    if (first_lookup_) {
        get_pose(prev_pose_x_, prev_pose_y_, quat);
        first_lookup_ = false;
    }

    curr_sliding_grid_ = std::vector<std::vector<int>>(window_height_, std::vector<int>(window_width_, -1));        
    double curr_pose_x, curr_pose_y;
    get_pose(curr_pose_x, curr_pose_y, quat);

    // Accounts for the adjusted shift as the robot moves forward, backward, left, right, etc.
    // This allows the cvStaticLayer "window" to always contain data around the robot position.
    int trans_cols = (curr_pose_x - prev_pose_x_) / resolution_;
    int trans_rows = (curr_pose_y - prev_pose_y_) / resolution_;
    grid_lock_.lock();
    for (int i = 0; i < window_height_; i++) {
        for (int j = 0; j < window_width_; j++) {

            int transformed_i = i + trans_rows;
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

    bool new_iteration = true;
    static int iteration = 0;
    if (debug_ && iteration % 10000 == 0) {
        RCLCPP_INFO(node->get_logger(), "2 x:%i", grid_x_);
        RCLCPP_INFO(node->get_logger(), "2 y:%i", grid_y_);
    }

    for (int i = 0; i < cv_height; i++) {
        for (int j = 0; j < cv_width; j++) {
            geometry_msgs::msg::Point camera_view;
            camera_view.y = cv_height - i - 1;
            camera_view.x = cv_width - j - 1;
            camera_view.z = 0;
            geometry_msgs::msg::Point cv_grid_location = cv_view_to_cv_grid(camera_view, new_iteration);
            if (debug_ && iteration % 10000 == 0) {
                RCLCPP_INFO(node->get_logger(), "x:%f", cv_grid_location.x);
                RCLCPP_INFO(node->get_logger(), "y:%f", cv_grid_location.y);
                iteration = 0;
            }
            int val = occ_grid->data[(cv_height - i - 1)*cv_width + (cv_width - j - 1)];
            if (val == 1) {
                val = 100;
            }
            else if (val == 0) {
                val = 0;
            }

            if (val >= 0) {
                int x = static_cast<int>(cv_grid_location.x);
                int y = static_cast<int>(cv_grid_location.y);

                if (debug_ && iteration % 10000 == 0) {
                    RCLCPP_INFO(node->get_logger(), "x grid:%i", x);
                    RCLCPP_INFO(node->get_logger(), "y grid:%i", y);
                    iteration = 0;
                }

                if ((y < window_height_ && y >= 0) && (x < window_width_ && x >= 0)) {                    
                    if (debug_ && iteration % 10000 == 0) {
                        RCLCPP_INFO(node->get_logger(), "assiging val%i", val);
                        iteration = 0;
                    }
                    curr_sliding_grid_[y][x] = val;
                }
            }
        }
        if (debug_) { iteration++; }
    }

    prev_sliding_grid_ = curr_sliding_grid_;
    grid_lock_.unlock();

    prev_pose_x_ =  curr_pose_x;
    prev_pose_y_ =  curr_pose_y;
}

geometry_msgs::msg::Point cvStaticLayer::cv_view_to_cv_grid(const geometry_msgs::msg::Point& coord, bool &new_iteration) {
    geometry_msgs::msg::Point transformed_point;
    bool transform_found = false;
    static geometry_msgs::msg::TransformStamped transform;
    if (!new_iteration) {
        tf2::doTransform(coord, transformed_point, transform);
        transformed_point.x += transform.transform.translation.x / resolution_;
        transformed_point.y += transform.transform.translation.y / resolution_;

        return transformed_point;
    }

    while(!transform_found) {
        try {
            transform = tf_buffer_->lookupTransform("cv_grid", "computer_vision_view", tf2::TimePointZero);
            transform_found = true;
            // RCLCPP_INFO(node->get_logger(), "cvStaticLayer to computer_vision_view transform found.");
        }
        catch (const tf2::TransformException& ex) {
            // RCLCPP_ERROR(node->get_logger(), "Failed to transform point: %s", ex.what());
        }
    }

    new_iteration = false;
    tf2::doTransform(coord, transformed_point, transform);
    transformed_point.x += transform.transform.translation.x / resolution_;
    transformed_point.y += transform.transform.translation.y / resolution_;
    return transformed_point;
}

geometry_msgs::msg::PoseStamped cvStaticLayer::get_cv_grid_origin() {
    geometry_msgs::msg::PoseStamped transformed_point;
    geometry_msgs::msg::PoseStamped input_point;
    input_point.pose.position.x = 0;
    input_point.pose.position.y = 0;
    input_point.pose.position.z = 0;
    input_point.pose.orientation.x = 0;
    input_point.pose.orientation.y = 0;
    input_point.pose.orientation.z = 0;
    input_point.pose.orientation.w = 1.0;
    bool transform_found = false;
    while(!transform_found) {
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("odom", "cv_grid", rclcpp::Time(0));
            tf2::doTransform(input_point, transformed_point, transform);
            transformed_point.header.frame_id = "odom";
            transformed_point.header.stamp = node->now();
            transform_found = true;
        }
        catch (const tf2::TransformException& ex) {
            // RCLCPP_ERROR(node->get_logger(), "Failed to transform point: %s", ex.what());
            // Handle the exception appropriately
        }
    }
    return transformed_point;
}

void cvStaticLayer::get_pose(double &pose_x, double &pose_y, geometry_msgs::msg::Quaternion& quat) {
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
        node->get_logger(), "Could not transform %s to %s: %s",
        toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        return;
    }

    pose_x = t.transform.translation.x;
    pose_y = t.transform.translation.y;
    quat = t.transform.rotation;
    return;
}

// void cvStaticLayer::updateBounds(double origin_x, double origin_y, double origin_yaw,
//                               double* min_x, double* min_y, double* max_x, double* max_y) {
    
// }

// void cvStaticLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
//     return;
// }


// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<cvStaticLayer>());
//     rclcpp::shutdown();
//     return 0;
// }
}

PLUGINLIB_EXPORT_CLASS(nav2_cv_static_layer::cvStaticLayer, nav2_costmap_2d::Layer)
