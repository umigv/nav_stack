#include "occupub/merge_costmap.h"

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
MergeService::MergeService() : Node("merge_service") {}

// Renamed the function to follow C++ naming conventions
void MergeService::occupancyGridSubscriber()
{
    cv_og_subscriber = create_subscription<OccupancyGrid>(
        "/cv_grid", 10, std::bind(&MergeService::populateCvOccupancyGrid, this, _1));
    
    sensors_occupancy_grid_subscriber = create_subscription<OccupancyGrid>(
        "/sen_grid", 10, std::bind(&MergeService::populateSensorsOccupancyGrid, this, _1));
}

OccupancyGrid MergeService::mergeSLAMAndLaneLine(const OccupancyGrid &cv_cm, const OccupancyGrid &slam_cm)
{
    /* -1 == unknown
        0 == non-drivable
        1 == drivable
    */

    // Create merged grid with the same dimensions as slam_cm
    OccupancyGrid merged_grid = slam_cm;

    // Define pose offsets
    pose_x;
    pose_y = cv_cm.info.height - pose_y;
 
    // Calculate offset for merging
    int dx = cv_cm.info.width / 2;
    int dy = cv_cm.info.height / 2;

    // Iterate through each cell in cv_cm
    for (size_t row = 0; row < cv_cm.info.height; ++row) {
        for (size_t col = 0; col < cv_cm.info.width; ++col) {
            // Calculate corresponding indices in slam_cm
            int slam_row = (row + pose_y - dy);
            int slam_col = (col + pose_x - dx);

            // Ensure the calculated indices are within bounds of slam_cm
            if (slam_row >= 0 && slam_row < static_cast<int>(slam_cm.info.height) &&
                slam_col >= 0 && slam_col < static_cast<int>(slam_cm.info.width)) {
                // Get values from both grids
                int slam_value = slam_cm.data[slam_row * slam_cm.info.width + slam_col];
                int lane_value = cv_cm.data[row * cv_cm.info.width + col];

                // Merge the values and update merged_grid
                merged_grid.data[slam_row * slam_cm.info.width + slam_col] = merge_cell<int8_t>(slam_value, lane_value);
            }
        }
    }

    return merged_grid;
}

void MergeService::printOccupancyGridInfo(const OccupancyGrid &grid)
{
    // Print information about the received message
    RCLCPP_INFO(get_logger(), "Received message. Height: %d, Width: %d", grid.info.height, grid.info.width);

    // Print data values from the received message
    for (size_t row = 0; row < grid.info.height; ++row) {
        for (size_t col = 0; col < grid.info.width; ++col) {
            int value = grid.data[row * grid.info.width + col];
            // if (value ==1){
            RCLCPP_INFO(get_logger(), "Value at (%zu, %zu): %d", row, col, value);
            // }
        }
    }
}

void MergeService::populateCvOccupancyGrid(const OccupancyGrid::SharedPtr cv_cm)
{
    cv_occupancy_grid = *cv_cm;  // Dereference the shared pointer and copy data
    // Additional logic if needed
    // Set a robotPose variable to the center of the grid
    robotPoseCV = std::make_pair(cv_occupancy_grid.info.width/2, cv_occupancy_grid.info.height/2);

    OccupancyGrid merged_grid = mergeSLAMAndLaneLine(cv_occupancy_grid, sensors_occupancy_grid);
    //print the CV pose
    //RCLCPP_INFO(get_logger(), "Robot pose: (%f, %f)", robotPoseCV.first, robotPoseCV.second);
    
     printOccupancyGridInfo(merged_grid);
    // Print data values from the received message
    // for (size_t row = 0; row < cv_occupancy_grid.info.height; ++row) {
    //     for (size_t col = 0; col < cv_occupancy_grid.info.width; ++col) {
    //         int value = cv_occupancy_grid.data[row * cv_occupancy_grid.info.width + col];
    //         RCLCPP_INFO(get_logger(), "Value at (%zu, %zu): %d", row, col, value);
    //     }
    // }
}

void MergeService::get_pose(double &pose_x, double &pose_y) {
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

void MergeService::populateSensorsOccupancyGrid(const OccupancyGrid::SharedPtr snsr_cm)
{
    sensors_occupancy_grid = *snsr_cm;  // Dereference the shared pointer and copy data
    // Additional logic if needed
    // initialize the robotPose variable
    get_pose(pose_x, pose_y);
    robotPoseSlam = std::make_pair(pose_x, pose_y);

    OccupancyGrid merged_grid = mergeSLAMAndLaneLine(cv_occupancy_grid, sensors_occupancy_grid);
    //print the SLAM pose
    //RCLCPP_INFO(get_logger(), "Robot pose: (%f, %f)", robotPoseSlam.first, robotPoseSlam.second);
     printOccupancyGridInfo(merged_grid);
    // Print data values from the received message
    // for (size_t row = 0; row < sensors_occupancy_grid.info.height; ++row) {
    //     for (size_t col = 0; col < sensors_occupancy_grid.info.width; ++col) {
    //         int value = sensors_occupancy_grid.data[row * sensors_occupancy_grid.info.width + col];
    //         RCLCPP_INFO(get_logger(), "Value at (%zu, %zu): %d", row, col, value);
    //     }
    // }
}


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
