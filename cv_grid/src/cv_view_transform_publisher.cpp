#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"




class cv_view_transform_publisher : public rclcpp::Node {
public:
    cv_view_transform_publisher() : Node("cv_view_transform_publisher") {
        resolution_ = 0.05;
        bool use_sim_time = false;
        this->get_parameter("use_sim_time", use_sim_time);
        cv_view_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        // timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&cv_view_transform_publisher::publish, this));
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10, std::bind(&cv_view_transform_publisher::cv_grid_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("computer_vision_view_grid", 10);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);      
    }
private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> cv_view_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
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
        publishGrid(occ_grid);
    }

    void publishGrid(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr occ_grid) {
        nav_msgs::msg::OccupancyGrid cv_view_grid;
        cv_view_grid.data = occ_grid->data;
        cv_view_grid.info = occ_grid->info;
        cv_view_grid.header.stamp = this->get_clock()->now();
        cv_view_grid.header.frame_id = "odom";
        // cv_view_grid.info.origin.position.x = 0;
        // cv_view_grid.info.origin.position.y = 0;
        // cv_view_grid.info.origin.position.z = 0;

        // // Orientation
        // cv_view_grid.info.origin.orientation.w = 1;
        // cv_view_grid.info.origin.orientation.x = 0;
        // cv_view_grid.info.origin.orientation.y = 0;
        // cv_view_grid.info.origin.orientation.z = 0;
        cv_view_grid.info.origin = get_cv_grid_origin().pose;
        publisher_->publish(cv_view_grid);
    }

    geometry_msgs::msg::PoseStamped get_cv_grid_origin() {
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
                geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("odom", "computer_vision_view", rclcpp::Time(0));
                tf2::doTransform(input_point, transformed_point, transform);
                transformed_point.header.frame_id = "odom";
                transformed_point.header.stamp = this->now();
                transform_found = true;
                // RCLCPP_INFO(this->get_logger(), "cv_grid to computer_vision_view transform found.");
            }
            catch (const tf2::TransformException& ex) {
                // RCLCPP_ERROR(this->get_logger(), "Failed to transform point: %s", ex.what());
                // Handle the exception appropriately
            }
        }
        return transformed_point;
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cv_view_transform_publisher>());
    rclcpp::shutdown();
    return 0;
}