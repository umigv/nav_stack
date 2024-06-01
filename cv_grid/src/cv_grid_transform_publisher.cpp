#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <utility>


class cv_grid_transform_publisher : public rclcpp::Node {
public:
    cv_grid_transform_publisher() : Node("cv_grid_transform_publisher"), window_height_(200), window_width_(200) {
        resolution_ = 0.05;
        bool use_sim_time = true;
        this->declare_parameter("Height", 200);
        this->declare_parameter("Width", 200);
        this->declare_parameter("Resolution", 0.05);

        this->get_parameter("use_sim_time", use_sim_time);
        this->get_parameter("Height", window_height_);
        this->get_parameter("Width", window_width_);
        this->get_parameter("Resolution", resolution_);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        cv_grid_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&cv_grid_transform_publisher::publish, this));
        
        RCLCPP_INFO(this->get_logger(), "Height %i", window_height_);   
    }

private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> cv_grid_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    int window_height_;
    int window_width_;
    double resolution_;

    void publish() {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "cv_grid";
        double rob_x;
        double rob_y;
        geometry_msgs::msg::Quaternion quat;
        get_pose(rob_x, rob_y, quat);

        // The current robot x coordinate in odom - the robot x coordinate in the grid * the grid resolution_.
        transform.transform.translation.x = (rob_x - window_width_ * 0.5 * resolution_);
        transform.transform.translation.y = (rob_y - window_height_ * 0.5 * resolution_);
        transform.transform.rotation.x = 0;
        transform.transform.rotation.y = 0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        transform.header.stamp = this->get_clock()->now();
        cv_grid_broadcaster_->sendTransform(transform);
        RCLCPP_INFO(this->get_logger(), "Cv Grid Transform published");
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

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cv_grid_transform_publisher>());
    rclcpp::shutdown();
    return 0;
}