#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav_msgs/msg/occupancy_grid.hpp"



class cv_view_transform_publisher : public rclcpp::Node {
public:
    cv_view_transform_publisher() : Node("cv_view_transform_publisher") {
        resolution_ = 0.05;
        bool use_sim_time = false;
        this->get_parameter("use_sim_time", use_sim_time);
        cv_grid_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        // timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&cv_view_transform_publisher::publish, this));
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10, std::bind(&cv_view_transform_publisher::cv_grid_callback, this, std::placeholders::_1));
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);       
    }
private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> cv_grid_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    double resolution_;

    void publish(int grid_x, int grid_y) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "cv_grid";
        double rob_x;
        double rob_y;
        geometry_msgs::msg::Quaternion quat;
        get_pose(rob_x, rob_y, quat);

        // The current robot x coordinate in odom - the robot x coordinate in the grid * the grid resolution_.
        transform.transform.translation.x = rob_x - grid_x * resolution_;
        transform.transform.translation.y = rob_y - grid_y * resolution_;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.w = 1.0;

        transform.header.stamp = this->get_clock()->now();
        cv_grid_broadcaster_->sendTransform(transform);
        RCLCPP_INFO(this->get_logger(), "Cv Grid Transform published");
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

void get_pose(double &pose_x, double &pose_y, geometry_msgs::msg::Quaternion& quat) {
    std::string fromFrameRel = "odom";
    std::string toFrameRel = "base_link";
    geometry_msgs::msg::TransformStamped t;

    // Look up for the transformation between target_frame and turtle2 frames
    // and send velocity commands for turtle2 to reach target_frame
    try {
        t = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        // RCLCPP_INFO(
        // this->get_logger(), "Could not transform %s to %s: %s",
        // toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        // rclcpp::sleep(0.1);
        return;
    }

    pose_x = t.transform.translation.x;
    pose_y = t.transform.translation.y;
    quat = t.transform.rotation;

    return;
}

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cv_view_transform_publisher>());
    rclcpp::shutdown();
    return 0;
}