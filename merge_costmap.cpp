#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using std::placeholders::_1;

class OccupancyGridSubscriber : public rclcpp::Node {
    public:
        OccupancyGridSubscriber() : Node("occupancy_grid_subscriber") {
        cv_og_subsriber_ = this->create_subscription<nav_msgs::OccupancyGrid>(
            "/occupancy_grid", 10, std::bind(&OccupancyGridSubscriber::populate_cv_occupancy_grid, this, _1)
        )

        sensors_occupancy_grid_subsriber = this->create_subscription<nav_msgs::OccupancyGrid>(
            "/occupancy_grid", 10, std::bind(&OccupancyGridSubscriber::populate_sensors_occupancy_grid, this, _1)
        )
    }


    private:
        // ------------- OCCUPANCY GRID VARIABLES -----------------

        // sensors occupancy grid variable
        nav_msgs::msg::OccupancyGrid sensors_occupancy_grid;

        // CV occupancy grid variable
        nav_msgs::msg::OccupancyGrid cv_occupancy_grid;

        // Combined Occupancy Grid variable
        nav_msgs::msg::OccupancyGrid combined_occupancy_grid;
        

        // ------------------------CALLBACKS -------------------------
        // populate cv occupancy grid
        void populate_cv_occupancy_grid(const nav_msgs::msg::OccupancyGrid::SharedPtr &cv_cm)
        {
            //Myabe save snsr metadata once to increase efficency - resolution, width, height, origin
            cv_occupancy_grid.data = cv_cm->data;
            cv_occupancy_grid.info = cv_cm->info;
            cv_occupancy_grid.header = cv_cm->header; //cv_cm??
        }

        // populate global occupancy grid
        void populate_sensors_occupancy_grid(const nav_msgs::msg::OccupancyGrid::SharedPtr &snsr_cm)
        {
            //Myabe save snsr metadata once to increase efficency - resolution, width, height, origin
            sensors_occupancy_grid.data = snsr_cm->data;
            sensors_occupancy_grid.info = snsr_cm->info;
            sensors_occupancy_grid.header = snsr_cm->header;
        }
}


// subscr to global OG, make a copy of it use that to init our OG
// writing the transform to get robot curr position ie. tf2
// overlay part - taking in curr position adn begin overlay