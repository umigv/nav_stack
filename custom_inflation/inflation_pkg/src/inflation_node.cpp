#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class OccupancyGridInflation : public rclcpp::Node {
public:
    OccupancyGridInflation() : Node("occupancy_grid_inflation") {
        sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/test_occ", 10, std::bind(&OccupancyGridInflation::mapCallback, this, std::placeholders::_1));
        pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/inflated_occ", 10);
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        auto inflated_grid = *msg;
        inflateObstacles(inflated_grid, 15, 0.75);
        pub_->publish(inflated_grid);
    }

    void inflateObstacles(nav_msgs::msg::OccupancyGrid &grid, int radius, double decrease_factor) {
        int width = grid.info.width;
        int height = grid.info.height;

        std::vector<int8_t> new_data = grid.data;

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                if (grid.data[y * width + x] > 50) { 
                    for (int dy = -radius; dy <= radius; dy++) {
                        for (int dx = -radius; dx <= radius; dx++) {
                            
                            int real_radius = std::sqrt(std::abs(dx) + std::abs(dy));
                            int new_value = int(grid.data[y * width + x])  * (std::pow(decrease_factor,real_radius)); 
                            int nx = x + dx;
                            int ny = y + dy;
                            if (nx >= 0 && ny >= 0 && nx < width && ny < height) {
                                new_data[ny * width + nx] = std::max(new_data[ny * width + nx], int8_t(new_value));
                            }
                        }
                    }
                }
            }
        }
        grid.data = new_data;
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyGridInflation>());
    rclcpp::shutdown();
    return 0;
}
    