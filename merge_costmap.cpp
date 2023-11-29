#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/OccupancyGrid.msg"

using std::placeholders::_1;

class OccupancyGridSubscriber : public rclcpp::Node {
    public:
        OccupancyGridSubscriber() : Node("occupancy_grid_subscriber") {
        subscription_ = this->create_subscription<nav_msgs::OccupancyGrid>(
            "/occupancy_grid", 10, std::bind(&OccupancyGridSubscriber::topic_callback, this, _1)
        )
    }

    private:
        // occupancy grid variable
        int8_t occupancy_grid[][];

        void topic_callback(const nav_msgs::OccupancyGrid &msg) const {
            occupancy_grid = msg.data;
        }
}