#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

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
        target_frame_ = this->declare_parameter<std::string>("base_link", "odom");

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    void publishGrid()
    {
        std::string fromFrameRel = target_frame_.c_str();
        std::string toFrameRel = "turtle2";
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

        double x_pos = t.transform.translation.x,
        double y_pos = t.transform.translation.y;

        // Create a sample occupancy grid (replace this with your logic to generate the grid)
        int width = 10;
        int height = 10;
        std::vector<int8_t> gridData(width * height, 0);  // initialize with zeros

        // Populate grid with random values (0 or 1)
        for (int i = 0; i < width * height; ++i)
        {
            gridData[i] = (rand() % 2) == 0 ? 0 : 1;
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

    void cv_grid_callback(const nav_msgs::OccupancyGridPtr &occ_grid) 
    {
        local_grid_ = occ_grid
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    rclcpp::Subscriber<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target_frame_;
    nav_msgs::OccupancyGrid::Ptr local_grid_;
    nav_msgs::OccupancyGrid::Ptr static_grid_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cv_grid>(););
    rclcpp::shutdown();
    return 0;
}
