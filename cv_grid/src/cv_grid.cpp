#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
// #include "geometry_msgs/msg/Tra"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
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
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Initialize sliding window lane lines grids
        window_height_ = 200;
        window_width_ = 200;
        curr_sliding_grid_ = std::vector<std::vector<int>>(window_height_, std::vector<int>(window_width_, -1));
        prev_sliding_grid_ = std::vector<std::vector<int>>(window_height_, std::vector<int>(window_width_, -1));
        first_lookup_ = true;
    }

private:
    /*
        This will be called in the grid publisher.

        Need to create and publish the transform for the window grid.
    */
    void window_grid_transform_publisher() 
    {
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
        RCLCPP_INFO(this->get_logger(), "Transform published");
    }

    /*
        This will be called in the grid callback.

        Creates a transform between baselink and incoming grid.
    */
    void cv_view_transform_publisher(const int grid_x_in, const int grid_y_in) 
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = "base_link";
        transform.child_frame_id = "computer_vision_view";
        // The current robot x coordinate in odom - the robot x coordinate in the grid * the grid RESOLUTION.
        transform.transform.translation.x = grid_x_in * RESOLUTION;
        transform.transform.translation.y = grid_y_in * RESOLUTION;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.w = 1.0;

        transform.header.stamp = this->now();
        broadcaster_->sendTransform(transform);
        RCLCPP_INFO(this->get_logger(), "Transform published");
    }

    void publishGrid()
    {
        // geometry_msgs::msg::TransformStamped rob_pose = get_node_options.lookupTransform('baselink', 'odom', rclcpp::Time(0));

        std::vector<int8_t> gridData(window_width_ * window_height_, -1);  // initialize with unknown
        double rob_x;
        double rob_y;
        geometry_msgs::msg::Quaternion quat;
        get_pose(rob_x, rob_y, quat);

        int grid_x_ = -1;
        int grid_y_ = -1;
        for (int i = 0; i < window_width_ * window_height_; ++i)
        {
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
        occupancyGridMsg.info.origin.position.x = rob_x - grid_x_ * RESOLUTION;
        occupancyGridMsg.info.origin.position.y = rob_y - grid_y_ * RESOLUTION;
        occupancyGridMsg.info.origin.position.z = 0.0;

        // Convert the 1D grid data to 2D
        occupancyGridMsg.data = gridData;

        // Publish the OccupancyGrid message
        publisher_->publish(occupancyGridMsg);
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

    /*
        When merging the maps:

        The current_sliding_grid will be in the same orientation as the odom frame. This does not change.

        The incoming occupancy grid, occ_grid, will be orientated in the same way as baselink.

        We will need to transform the orientation of cells in the incoming occupancy grid to those of cells in the current_sliding_grid.
    */
    void cv_grid_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr occ_grid) 
    {   
        geometry_msgs::msg::Quaternion quat;
        if (first_lookup_) {
            get_pose(prev_pose_x_, prev_pose_y_, quat);
            first_lookup_ = false;
        }

        curr_sliding_grid_ = std::vector<std::vector<int>>(window_height_, std::vector<int>(window_width_, -1));        
        

        double curr_pose_x, curr_pose_y;
        get_pose(curr_pose_x, curr_pose_y, quat);

        int robot_row = window_height_/2;
        int robot_col = window_width_/2; 

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
        for (int i = 0; i < cv_height; i++) 
        {
            for (int j = 0; j < cv_width; j++)
            {
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

        cv_view_transform_publisher(grid_x_, grid_y_);


        for (int i = 0; i < cv_height; i++) 
        {
            for (int j = 0; j < cv_width; j++)
            {
                int val = occ_grid->data[(cv_height - i - 1)*cv_width + (cv_width - j - 1)];
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

    geometry_msgs::msg::PointStamped cv_view_to_cv_grid(const geometry_msgs::msg::PointStamped& coord) 
    {
        geometry_msgs::msg::PointStamped transformed_point;
        try
        {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("cv_grid", "computer_vision_view", rclcpp::Time(0));
            tf2::doTransform(coord, transformed_point, transform);
            transformed_point.header.frame_id = "cv_grid";
            transformed_point.header.stamp = this->now();
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to transform point: %s", ex.what());
            // Handle the exception appropriately
        }
        return transformed_point;
    }

    /*
        Returns the origin of the cv_grid in the odom frame.
    */
    geometry_msgs::msg::TransformStamped get_cv_grid_origin() 
    {
        geometry_msgs::msg::TransformStamped transformed_point;
        geometry_msgs::msg::TransformStamped input_point;
        input_point.transform.translation.x = 0;
        input_point.transform.translation.y = 0;
        input_point.transform.translation.z = 0;
        input_point.transform.rotation.x = 0;
        input_point.transform.rotation.y = 0;
        input_point.transform.rotation.z = 0;
        input_point.transform.rotation.w = 1.0;

        try
        {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("odom", "cv_grid", rclcpp::Time(0));
            tf2::doTransform(input_point, transformed_point, transform);
            transformed_point.header.frame_id = "odom";
            transformed_point.header.stamp = this->now();
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to transform point: %s", ex.what());
            // Handle the exception appropriately
        }
        return transformed_point;
    }


    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
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
    bool first_lookup_;
    int grid_x_ = -1;
    int grid_y_ = -1;
    const double RESOLUTION = 0.05;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cv_grid>());
    rclcpp::shutdown();
    return 0;
}
