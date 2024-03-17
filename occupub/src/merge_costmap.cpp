#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>  // Fix: Corrected the include statement
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using TransformStamped = geometry_msgs::msg::TransformStamped;  // Fix: Added semicolon

// Forward declaration
class TF2sub;

// Subscribes to TF2 and updates global position inside the occupancy grid
class TF2sub : public rclcpp::Node
{
public:
    float current_x_pos = 0.0;
    float current_y_pos = 0.0;

    // Create a subscription to the "/tf_world" topic with a callback function
    TF2sub() : Node("tf2_world_subscriber")
    {
        tf2_subscriber = create_subscription<TransformStamped>(
            "/tf_world", 10, std::bind(&TF2sub::tf2WorldCallback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<TransformStamped>::SharedPtr tf2_subscriber;

    // Callback function to process received TF2 transforms and populate global variables
    void tf2WorldCallback(const TransformStamped::SharedPtr msg)
    {
        // Update current position
        current_x_pos = msg->transform.translation.x;
        current_y_pos = msg->transform.translation.y;
    }
};

// Subscribes to the global occupancy grid and the CV occupancy grid
// Merges them together to create a new global occupancy grid
// That is then published to the global costmap
class MergeService : public rclcpp::Node
{
public:
    MergeService() : Node("merge_service") {}

    // Renamed the function to follow C++ naming conventions
    void occupancyGridSubscriber()
    {
        cv_og_subscriber = create_subscription<OccupancyGrid>(
            "/cv_grid", 10, std::bind(&MergeService::populateCvOccupancyGrid, this, _1));
        
        sensors_occupancy_grid_subscriber = create_subscription<OccupancyGrid>(
            "/sen_grid", 10, std::bind(&MergeService::populateSensorsOccupancyGrid, this, _1));
    }

    OccupancyGrid mergeSLAMAndLaneLine(const OccupancyGrid &cv_cm, const OccupancyGrid &slam_cm)
    {
         /*  -1 == unknown
                0 == non-drivable
                1 == drivable
            */

            // the two cost maps are the same size
            OccupancyGrid merged_grid = slam_cm;

            // start merge
            for (size_t row = 0; row < slam_cm.info.height; ++row) {
                for (size_t col = 0; col < slam_cm.info.width; ++col) {
                    // get the corresponding values from the 2 cost maps
                    int slam_value = slam_cm.data[row * slam_cm.info.width + col];
                    int lane_value = cv_cm.data[row * cv_cm.info.width + col];
                    
                    /*
                    9 cases:
                        LL = -1, SLAM = -1 -> -1
                        LL = -1, SLAM = 0 -> 0
                        LL = -1, SLAM = 1 -> 1
                        LL = 0, SLAM = -1 -> 0
                        LL = 0, SLAM = 0 -> 0
                        LL = 0, SLAM = 1 -> 1 OLD: 0
                        LL = 1, SLAM = -1 -> 1
                        LL = 1, SLAM = 0 -> 1 OLD: 0
                        LL = 1, SLAM = 1 -> 1
                    */
                    if (lane_value == -1){
                        if (slam_value == -1){
                            merged_grid.data[row * slam_cm.info.width + col] = -1;
                        }
                        else if (slam_value == 0){
                            merged_grid.data[row * slam_cm.info.width + col] = 0;
                        }
                        else if (slam_value == 1){
                            merged_grid.data[row * slam_cm.info.width + col] = 1;
                        }
                    }
                    if (lane_value == 0){
                        if (slam_value == -1){
                            merged_grid.data[row * slam_cm.info.width + col] = 0;
                        }
                        else if (slam_value == 0){
                            merged_grid.data[row * slam_cm.info.width + col] = 0;
                        }
                        else if (slam_value == 1){
                            merged_grid.data[row * slam_cm.info.width + col] = 1;
                        }
                    }
                    if (lane_value == 1){
                        if (slam_value == -1){
                            merged_grid.data[row * slam_cm.info.width + col] = 1;
                        }
                        else if (slam_value == 0){
                            merged_grid.data[row * slam_cm.info.width + col] = 1;
                        }
                        else if (slam_value == 1){
                            merged_grid.data[row * slam_cm.info.width + col] = 1;
                        }
                    }
                }
            }

            return merged_grid;
    }

    void printOccupancyGridInfo(const OccupancyGrid &grid)
    {
        // Print information about the received message
        RCLCPP_INFO(get_logger(), "Received message. Height: %d, Width: %d", grid.info.height, grid.info.width);

        // Print data values from the received message
        for (size_t row = 0; row < grid.info.height; ++row) {
            for (size_t col = 0; col < grid.info.width; ++col) {
                int value = grid.data[row * grid.info.width + col];
                if (value ==1){
                RCLCPP_INFO(get_logger(), "Value at (%zu, %zu): %d", row, col, value);
                }
            }
        }
    }

private:
    rclcpp::Subscription<OccupancyGrid>::SharedPtr cv_og_subscriber;
    rclcpp::Subscription<OccupancyGrid>::SharedPtr sensors_occupancy_grid_subscriber;

    OccupancyGrid cv_occupancy_grid;
    OccupancyGrid sensors_occupancy_grid;

    void populateCvOccupancyGrid(const OccupancyGrid::SharedPtr cv_cm)
    {
        cv_occupancy_grid = *cv_cm;  // Dereference the shared pointer and copy data
        // Additional logic if needed
        // Print information about the received message
        // RCLCPP_INFO(get_logger(), "Received CV occupancy grid. Height: %d, Width: %d", cv_occupancy_grid.info.height, cv_occupancy_grid.info.width);
        OccupancyGrid merged_grid = mergeSLAMAndLaneLine(cv_occupancy_grid, sensors_occupancy_grid);
        printOccupancyGridInfo(merged_grid);
        // Print data values from the received message
        // for (size_t row = 0; row < cv_occupancy_grid.info.height; ++row) {
        //     for (size_t col = 0; col < cv_occupancy_grid.info.width; ++col) {
        //         int value = cv_occupancy_grid.data[row * cv_occupancy_grid.info.width + col];
        //         RCLCPP_INFO(get_logger(), "Value at (%zu, %zu): %d", row, col, value);
        //     }
        // }
    }

    void populateSensorsOccupancyGrid(const OccupancyGrid::SharedPtr snsr_cm)
    {
        sensors_occupancy_grid = *snsr_cm;  // Dereference the shared pointer and copy data
        // Additional logic if needed
        // Print information about the received message
        // RCLCPP_INFO(get_logger(), "Received sensors occupancy grid. Height: %d, Width: %d", sensors_occupancy_grid.info.height, sensors_occupancy_grid.info.width);
        OccupancyGrid merged_grid = mergeSLAMAndLaneLine(cv_occupancy_grid, sensors_occupancy_grid);
        printOccupancyGridInfo(merged_grid);
        // Print data values from the received message
        // for (size_t row = 0; row < sensors_occupancy_grid.info.height; ++row) {
        //     for (size_t col = 0; col < sensors_occupancy_grid.info.width; ++col) {
        //         int value = sensors_occupancy_grid.data[row * sensors_occupancy_grid.info.width + col];
        //         RCLCPP_INFO(get_logger(), "Value at (%zu, %zu): %d", row, col, value);
        //     }
        // }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create instances of both subscriber and publisher
    auto tf2_subscriber = std::make_shared<TF2sub>();
    auto merge_service = std::make_shared<MergeService>();

    // Call the occupancyGridSubscriber function
    merge_service->occupancyGridSubscriber();

    // Spin both nodes
    //rclcpp::spin(tf2_subscriber);
    rclcpp::spin(merge_service);

    rclcpp::shutdown();
    return 0;
}
