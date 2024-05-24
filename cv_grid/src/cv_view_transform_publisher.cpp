#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/occupancy_grid.hpp"



class cv_view_transform_publisher : public rclcpp::Node {
public:
    cv_view_transform_publisher() : Node("cv_view_transform_publisher") {
        resolution_ = 0.05;
        bool use_sim_time = false;
        this->get_parameter("use_sim_time", use_sim_time);
        cv_view_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        // timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&cv_view_transform_publisher::publish, this));
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10, std::bind(&cv_view_transform_publisher::cv_grid_callback, this, std::placeholders::_1));
    }
private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> cv_view_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;

    double resolution_;

    void publish(int grid_x, int grid_y) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = "base_link";
        transform.child_frame_id = "computer_vision_view";
        // The current robot x coordinate in odom - the robot x coordinate in the grid * the grid resolution_.
        // transform.transform.translation.x = grid_x_in * resolution_;
        // transform.transform.translation.y = grid_y_in * resolution_;
        transform.transform.translation.x = -grid_x * resolution_;
        transform.transform.translation.y = grid_y * resolution_;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.w = 1.0;

        transform.header.stamp = this->get_clock()->now();
        cv_view_broadcaster_->sendTransform(transform);
        static int i = 0;
        if (i == 1000000) {
            RCLCPP_INFO(this->get_logger(), "Computer Vision View Transform published");
            i = 0;
        }
    }

    void cv_grid_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr occ_grid) {
        int cv_width = occ_grid->info.width;
        int cv_height = occ_grid->info.height;
        int grid_x = -1;
        int grid_y = -1;

        // Need to find where the 2 is in the occ_grid.
        for (int i = 0; i < cv_height; i++) {
            for (int j = 0; j < cv_width; j++) {
                if (occ_grid->data[(cv_height - i - 1)*cv_width + (cv_width - j - 1)] == 2) {
                    grid_x = cv_width - j - 1;
                    grid_y = cv_height - i - 1;
                    break;                
                } 
            }
            if (grid_x != -1) {
                break;
            }
        }

        publish(grid_x, grid_y);
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cv_view_transform_publisher>());
    rclcpp::shutdown();
    return 0;
}