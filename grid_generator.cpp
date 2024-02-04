#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class OccupancyGridGenerator : public rclcpp::Node
{
public:
    OccupancyGridGenerator() : Node("occupancy_grid_generator")
    {
        publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10);
        timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&OccupancyGridGenerator::publishGrid, this));
    }

private:
    void publishGrid()
    {
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

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OccupancyGridGenerator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}