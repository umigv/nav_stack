#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
// #include "geometry_msgs/msg/Tra"
#include <vector>

namespace CELL_CONSTS 
{
    constexpr unsigned int FREE_SPACE = 0;
    constexpr unsigned int LETHAL_OBSTACLE = 1;
    constexpr unsigned int CV_ROBOT_POS = 2;
    constexpr int UNKOWN = -1;
}

using namespace std::chrono_literals;

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
            "occupancy_grid", 10, std::bind(&cv_grid::cv_grid_callback, this, std::placeholders::_1));
    
        // Declare and acquire `target_frame` parameter
        target_frame_ = this->declare_parameter<std::string>("target_frame", "odom");

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);         

        // Initialize sliding window lane lines grids
        window_height_ = 200;
        window_width_ = 200;
        curr_sliding_grid_ = std::vector<std::vector<int>>(window_height_, std::vector<int>(window_width_, -1));
        prev_sliding_grid_ = std::vector<std::vector<int>>(window_height_, std::vector<int>(window_width_, -1));
        
        // Static grid (aka total competition size)
        // int comp_width = 60 / .05 // m / (m/cell)
        // int comp_height = 60 / .05 // m / (m/cell)

        // static_grid_ = std::vector<std::vector<int>>(comp_height, std::vector<int>(comp_width));
        first_lookup = true;
    }

private:
    void publishGrid()
    {
        // geometry_msgs::msg::TransformStamped rob_pose = get_node_options.lookupTransform('baselink', 'odom', rclcpp::Time(0));

        std::vector<int8_t> gridData(window_width_ * window_height_, -1);  // initialize with unknown
        double rob_x;
        double rob_y;
        geometry_msgs::msg::Quaternion quat;
        get_pose(rob_x, rob_y, quat);

        int grid_x = -1;
        int grid_y = -1;
        for (int i = 0; i < window_width_ * window_height_; ++i)
        {
            gridData[i] = curr_sliding_grid_[i / window_width_][i % window_width_];
            if (gridData[i] == 2) {
                std::cout << "found robot pose." << std::endl;
                grid_x = i % window_width_;
                grid_y = i / window_width_;
            }
        }

        // Create the OccupancyGrid message
        auto occupancyGridMsg = nav_msgs::msg::OccupancyGrid();
        occupancyGridMsg.header.stamp = now();
        occupancyGridMsg.header.frame_id = "odom";
        occupancyGridMsg.info.width = window_width_;
        occupancyGridMsg.info.height = window_height_;
        occupancyGridMsg.info.resolution = 0.05;  // Replace with your desired resolution
        occupancyGridMsg.info.origin.position.x = rob_x - grid_x * occupancyGridMsg.info.resolution;
        occupancyGridMsg.info.origin.position.y = rob_y - grid_y * occupancyGridMsg.info.resolution;
        occupancyGridMsg.info.origin.position.z = 0.0;
        // occupancyGridMsg.info.origin.orientation = quat;

        // geometry_msgs::msg::Quaternion rot;
        // rot.setRPY(0, 0, M_PI_2);
        // occupancyGridMsg.info.origin.orientation = rot * occupancyGridMsg.info.origin.orientation;



        // Convert the 1D grid data to 2D
        occupancyGridMsg.data = gridData;

        // Publish the OccupancyGrid message
        publisher_->publish(occupancyGridMsg);
        // RCLCPP_INFO(get_logger(), "Occupancy grid published");
    }
    void get_pose(double &pose_x, double &pose_y, geometry_msgs::msg::Quaternion& quat) {
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
    void cv_grid_callback(const nav_msgs::msg::OccupancyGrid::Ptr occ_grid) 
    {   
        geometry_msgs::msg::Quaternion quat;
        if (first_lookup) {
            get_pose(prev_pose_x_, prev_pose_y_, quat);
            first_lookup = false;
        }
        // auto occupancyGridMsg = nav_msgs::msg::OccupancyGrid();
        // occupancyGridMsg.header.stamp = now();
        // occupancyGridMsg.header.frame_id = "map";
        // occupancyGridMsg.info.width = occ_grid->info.width;
        // occupancyGridMsg.info.height = occ_grid->info.height;
        // occupancyGridMsg.info.resolution = occ_grid->info.resolution;
        // occupancyGridMsg.data = occ_grid->data;
        // publisher_->publish(occupancyGridMsg);

        curr_sliding_grid_ = std::vector<std::vector<int>>(window_height_, std::vector<int>(window_width_, -1));        
        // int resolution = occ_grid->info.resolution;
        double resolution = 0.05;

        double curr_pose_x, curr_pose_y;
        // curr_pose_x = prev_pose_x_ + 0;
        // curr_pose_y = prev_pose_y_ + 0.5;
        get_pose(curr_pose_x, curr_pose_y, quat);

        int robot_row = window_height_/2;
        int robot_col = window_width_/2; 

        // int robot_row = 0;
        // int robot_col = 0; 
        // std::cout << curr_pose_x << std::endl;
        // std::cout << curr_pose_y << std::endl; 
        // std::cout << prev_pose_x_ << std::endl;
        // std::cout << prev_pose_y_ << std::endl; 
        int trans_cols = (curr_pose_x - prev_pose_x_) / resolution;
        int trans_rows = (curr_pose_y - prev_pose_y_) / resolution;
  
        for (int i = 0; i < window_height_; i++) {
            for (int j = 0; j < window_width_; j++) {

                // To move up in grid, need our index to be smaller.
                
                // TODO Convert to actual code
                // theta = quat_to_euler(q)
                // xy_homo = [i j 1]
                // trans_mat = [[cos(theta) -sin(theta) trans_cols],
                //             [sin(theta) cos(theta) trans_rows],
                //             [    0           0         1]]
                // trans_xy = xy_homo * trans_mat //matrix multiply
                // xy_homo = xy_homo/xy_homo[2]


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

        for (int i = 0; i < cv_height; i++) 
        {
            for (int j = 0; j < cv_width; j++)
            {
                int val = occ_grid->data[(cv_height - i - 1)*cv_width + (cv_width - j - 1)];
                // std::cout << val << " ";
                // curr_sliding_grid_[robot_row + i][robot_col - cv_width/2 + j] = occ_grid->data[i*cv_width + j];
                if (val == 1)
                {
                    val = 100;
                }
                else if (val == 0)
                {
                    val = 0;
                }

                if (val >= 0)
                {
                    curr_sliding_grid_[robot_row + i][robot_col - cv_width/2 + j] = val;
                }
            }
            // std::cout << std::endl;
        }

        prev_sliding_grid_ = curr_sliding_grid_;

        prev_pose_x_ =  curr_pose_x;
        prev_pose_y_ =  curr_pose_y;

    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target_frame_;
    std::vector<std::vector<int>> curr_sliding_grid_;
    std::vector<std::vector<int>> prev_sliding_grid_;
    std::vector<std::vector<int>> static_grid_;
    int window_height_;
    int window_width_;
    double prev_pose_x_;
    double prev_pose_y_;
    bool first_lookup;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cv_grid>());
    rclcpp::shutdown();
    return 0;
}
