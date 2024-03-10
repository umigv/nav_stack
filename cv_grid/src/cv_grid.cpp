#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <vector>

class cv_grid : public rclcpp::Node
{
public:
    cv_grid() : Node("cv_grid")
    {
        // publish piece of global lane lines occupancy grid
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("cv_grid_out", 10);
        timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&cv_grid::publishGrid, this));
    
        // subscribe to local lane lines occupancy grid
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "cv_grid_in", 10, std::bind(&cv_grid::cv_grid_callback, this, _1));
    
        // Declare and acquire `target_frame` parameter
        target_frame_ = this->declare_parameter<std::string>("target_frame", "odom");

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize sliding window lane lines grids
        window_height_ = 200;
        window_width_ = 200;
        curr_sliding_grid_ = std::vector<std::vector<int>>(window_height_, std::vector<int>(window_width_));
        prev_sliding_grid_ = std::vector<std::vector<int>>(window_height_, std::vector<int>(window_width_));
        get_pose(prev_pose_x_, prev_pose_y_);
    }

private:
    void publishGrid()
    {
        int global_pose_x;
        int global_pose_y;
        get_pose(global_pose_x, global_pose_y);

        // Create a sample occupancy grid (replace this with your logic to generate the grid)
        int width = 10;
        int height = 10;
        std::vector<int8_t> gridData(width * height, 0);  // initialize with zeros

        // Populate grid with random values (0 or 1)
        for (int i = 0; i < width * height; ++i)
        {
            gridData[i] = rand() % 3 == 0 ? 0 : (rand() % 2 == 0 ? 1 : -1);
        }

        // Create the OccupancyGrid message
        auto occupancyGridMsg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        occupancyGridMsg->header.stamp = now();
        occupancyGridMsg->header.frame_id = "map";
        occupancyGridMsg->info.width = width;
        occupancyGridMsg->info.height = height;
        occupancyGridMsg->info.resolution = 0.05;  // Replace with your desired resolution
        occupancyGridMsg->info.origin.position.x = 0.0;
        occupancyGridMsg->info.origin.position.y = 0.0;
        occupancyGridMsg->info.origin.position.z = 0.0;

        // Convert the 1D grid data to 2D
        occupancyGridMsg->data = gridData;

        // Publish the OccupancyGrid message
        publisher_->publish(occupancyGridMsg);
        RCLCPP_INFO(get_logger(), "Occupancy grid published");
    }
    void get_pose(double &pose_x, double &pose_y) {
        std::string fromFrameRel = target_frame_.c_str();
        std::string toFrameRel = "base_link";
        geometry_msgs::msg::TransformStamped t;

        // Look up for the transformation between target_frame and turtle2 frames
        // and send velocity commands for turtle2 to reach target_frame
        try {
          t = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }

        pose_x = t.transform.translation.x;
        pose_y = t.transform.translation.y;
        
        return;
    }
    void cv_grid_callback(const nav_msgs::OccupancyGridPtr &occ_grid) 
    {
        curr_sliding_grid_ = std::vector<std::vector<int>>(window_height_, std::vector<int>(window_width_));        
        int resolution = occ_grid->info.resolution;

        int curr_pose_x, curr_pose_y;
        get_pose(curr_pose_x, curr_pose_y);

        int trans_cols = (curr_pose_x - prev_pose_x_) / resolution;
        int trans_rows = (curr_pose_y - prev_pose_y_) / resolution;
        for (int i = 0; i < window_width_; i++) {
            for (int j = 0; j < window_height_; j++) {
                int transformed_i = i + trans_rows;
                int transformed_j = j + trans_cols;
                if (transformed_i >= 0 && transformed_j >= 0 && transformed_i < window_height_ 
                        && transformed_j < window_width_) {
                    curr_sliding_grid_[transformed_i][transformed_j] = prev_sliding_grid_[i][j];
                }
            }
        }

        int cv_height = 50;
        int cv_width = 100;
        int top_left_i = window_height_/2 - cv_height_;
        int top_left_j = window_width_/2 - cv_width_/2;
        




        int global_origin_x = occ_grid->info.origin.position.x;
        int global_origin_y = occ_grid->info.origin.position.y;


        int width = occ_grid->info.width;
        int height = occ_grid->info.height;

        int grid_origin_x = (global_origin_x - origin_x_) / resolution;
        int grid_origin_y = (global_origin_y - origin_y_) / resolution;

        for (int i = height; i > 0; i--) {
            for (int j = 0; j < width; j++) {
                static_grid_[grid_origin_x+j][grid_origin_y - i] = occ_grid->data[width*i + j]
            }
        }
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    rclcpp::Subscriber<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target_frame_;
    nav_msgs::OccupancyGrid::Ptr local_grid_;
    std::vector<std::vector<int>> curr_sliding_grid_;
    std::vector<std::vector<int>> prev_sliding_grid_;
    int window_height_;
    int window_width_;
    double prev_pose_x_;
    double prev_pose_y_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cv_grid>(););
    rclcpp::shutdown();
    return 0;
}
