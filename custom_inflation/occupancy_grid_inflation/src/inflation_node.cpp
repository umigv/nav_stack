#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_interfaces/srv/inflation_grid.hpp"     // CHANGE

class OccupancyGridInflation : public rclcpp::Node {
public:
    OccupancyGridInflation() : Node("occupancy_grid_inflation") {
        sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/test_occ", 10, std::bind(&OccupancyGridInflation::mapCallback, this, std::placeholders::_1));
        pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/inflated_occ", 10);

        srv_ = this->create_service<map_interfaces::srv::InflationGrid>(
            "inflation_grid_service", 
            std::bind(&OccupancyGridInflation::handleInflationRequest, this, std::placeholders::_1, std::placeholders::_2));
        
        latest_grid_ = nullptr;
         // if you change this name make sure you also change the goal_selection_node  inflation_grid_service
    }


private:
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_grid_;  // Store the latest received grid
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        auto inflated_grid = *msg;
        inflateObstacles(inflated_grid, 15, 0.75);
        pub_->publish(inflated_grid);
        latest_grid_ = msg;
    }

    void handleInflationRequest(
        const std::shared_ptr<map_interfaces::srv::InflationGrid::Request> request,
        std::shared_ptr<map_interfaces::srv::InflationGrid::Response> response) 
    {
        if (!latest_grid_) {
            RCLCPP_WARN(this->get_logger(), "No occupancy grid received yet!");
            return;
        }

        auto inflated_grid = *latest_grid_;  
        inflateObstacles(inflated_grid, 15, 0.75);  // Inflate obstacles
        pub_->publish(inflated_grid);  // Publish inflated map

        response->occupancy_grid = inflated_grid;
        response->robot_pose_x = 50;  // Placeholder values
        response->robot_pose_y = 50;
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
    rclcpp::Service<map_interfaces::srv::InflationGrid>::SharedPtr srv_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyGridInflation>());
    rclcpp::shutdown();
    return 0;
}
    