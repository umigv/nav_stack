#include "merge_costmap.h"

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

    // create a robotPose index pair variable
    std::pair<double, double> robotPoseCV;
    std::pair<double, double> robotPoseSlam;
    double pose_x, pose_y;

    // Declare and acquire `target_frame` parameter
    target_frame_ = this->declare_parameter<std::string>("target_frame", "odom");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
         /* -1 == unknown
            0 == non-drivable
            1 == drivable
        */

        // the two cost maps are the same size
        OccupancyGrid merged_grid = slam_cm;

        // start merge
        for (size_t row = 0; row < slam_cm.info.height; ++row) {
            for (size_t col = 0; col < slam_cm.info.width; ++col) {
                // get the corresponding values from the 2 cost maps
                auto slam_value = slam_cm.data[row * slam_cm.info.width + col];
                auto lane_value = cv_cm.data[row * cv_cm.info.width + col];
                merged_grid.data.at(row * slam_cm.info.width + col) = merge_cell<int8_t>(row, col, merged_grid.data);
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

    std::string target_frame_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


    void populateCvOccupancyGrid(const OccupancyGrid::SharedPtr cv_cm)
    {
        cv_occupancy_grid = *cv_cm;  // Dereference the shared pointer and copy data
        // Additional logic if needed
        // Set a robotPose variable to the center of the grid
        robotPoseCV = std::make_pair(cv_occupancy_grid.info.width/2, cv_occupancy_grid.info.height/2);

        OccupancyGrid merged_grid = mergeSLAMAndLaneLine(cv_occupancy_grid, sensors_occupancy_grid);
        //print the CV pose
        RCLCPP_INFO(get_logger(), "Robot pose: (%f, %f)", robotPoseCV.first, robotPoseCV.second);
        
        // printOccupancyGridInfo(merged_grid);
        // Print data values from the received message
        // for (size_t row = 0; row < cv_occupancy_grid.info.height; ++row) {
        //     for (size_t col = 0; col < cv_occupancy_grid.info.width; ++col) {
        //         int value = cv_occupancy_grid.data[row * cv_occupancy_grid.info.width + col];
        //         RCLCPP_INFO(get_logger(), "Value at (%zu, %zu): %d", row, col, value);
        //     }
        // }
    }

    void get_pose(double &pose_x, double &pose_y) {
        std::string fromFrameRel = target_frame_.c_str();
        std::string toFrameRel = "base_link";
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

        pose_x = t.transform.translation.x;
        pose_y = t.transform.translation.y;
        
        return;
    }

    void populateSensorsOccupancyGrid(const OccupancyGrid::SharedPtr snsr_cm)
    {
        sensors_occupancy_grid = *snsr_cm;  // Dereference the shared pointer and copy data
        // Additional logic if needed
        // initialize the robotPose variable
        robotPoseSlam = std::make_pair(pose_x, pose_y);

        OccupancyGrid merged_grid = mergeSLAMAndLaneLine(cv_occupancy_grid, sensors_occupancy_grid);
        //print the SLAM pose
        RCLCPP_INFO(get_logger(), "Robot pose: (%f, %f)", robotPoseSlam.first, robotPoseSlam.second);
        // printOccupancyGridInfo(merged_grid);
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
