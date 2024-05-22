#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"

class cv_view_transform_publisher : public rclcpp::Node {
public:
    cv_view_transform_publisher() : Node("cv_view_transform_publisher") {
        bool use_sim_time = false;
        this->get_parameter("use_sim_time", use_sim_time);
        cv_view_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&cv_view_transform_publisher::publish, this));
    }
private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> cv_view_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish() {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = "base_link";
        transform.child_frame_id = "computer_vision_view";
        // The current robot x coordinate in odom - the robot x coordinate in the grid * the grid resolution_.
        // transform.transform.translation.x = grid_x_in * resolution_;
        // transform.transform.translation.y = grid_y_in * resolution_;
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
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
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cv_view_transform_publisher>());
    rclcpp::shutdown();
    return 0;
}