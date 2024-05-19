#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace CELL_CONSTS 
{
    constexpr unsigned int FREE_SPACE = 0;
    constexpr unsigned int LETHAL_OBSTACLE = 1;
    constexpr unsigned int CV_ROBOT_POS = 2;
    constexpr int UNKOWN = -1;
}

class cv_grid : public rclcpp::Node
{
public:
    cv_grid();

private:
    // Functions

    /*
        Publishes the occupancy grid that combines the incomming Computer Vision occupancy grid,
        with the merged lane line occupancy built and maintained by cv_grid.
    */
    void publishGrid();
    
    /*
        When merging the maps:
            The current_sliding_grid will be in the same orientation as the odom frame. This does not change.

            The incoming occupancy grid, occ_grid, will be orientated in the same way as baselink.

            We will need to transform the orientation of cells in the incoming occupancy grid to those 
            of cells in the current_sliding_grid.
    */
    void cv_grid_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr occ_grid);
    
    /*
        This will be called in the grid publisher.

        Need to create and publish the transform for the window grid.
    */
    void window_grid_transform_publisher();

    /*
        This will be called in the grid callback.

        Creates a transform between baselink and incoming grid.
    */
    void cv_view_transform_publisher();
    
    /*
        Returns the transform from the computer vision occupancy grid view to the cv_grid (the one outputted by this node).
    */
    geometry_msgs::msg::Point cv_view_to_cv_grid(const geometry_msgs::msg::Point& coord);

    /*
        Returns the origin of the cv_grid in the odom frame.
    */
    geometry_msgs::msg::PoseStamped get_cv_grid_origin();

    /*
        Returns the robot's position in the target_frame_. Default is "odom" therefore returns baselink 
        to odom.
    */
    void get_pose(double &pose_x, double &pose_y, geometry_msgs::msg::Quaternion& quat);

    // Variables
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