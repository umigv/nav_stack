#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using std::placeholders::_1;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
// using Point = geometry_msgs::msg::Point;
using x = TF2sub::current_x_pos;
using y = TF2sub::current_y_pos;
using TransformStamped = geometry_msgs::msg::TransformStamped

//Subscribes to TF2 and update global position inside occupancy grid
class TF2sub : public rclcpp::Node
{
    public:
        // ------------ CURRENT LOCATION VARIABLES ------------
        float current_x_pos = 0.0;
        float current_y_pos = 0.0;
        
        // Create a subscription to the "/tf_world" topic with a callback function
        TF2sub() : Node("tf2_world_subscriber")
        {
            tf2_subscriber = create_subscription<TransformStamped>(
                "/tf_world", 10, std::bind(&TF2sub::tf2WorldCallback, this, std::placeholders::_1));
        }
        
    private:
        // Callback function to process received TF2 transforms
        // populate global variables
        void tf2WorldCallback(const TransformStamped::SharedPtr msg)
        {
            // Update current position
            current_x_pos = msg->transform.translation.x;
            current_y_pos = msg->transform.translation.y;
        }
        // Subscription object for the "/tf_world" topic
        rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription_;
};

// Subsribes to the global occupancy grid and the cv occupancy grid 
// and merges them together to create a new global occupancy grid
// that is then published to the global costmap
class MergeService : public rclcpp::Node {
    public:
        MergeService() : Node("merge_service") {}

        OccupancyGridSubscriber() : Node("occupancy_grid_subscriber") {
            cv_og_subsriber = this->create_subscription<nav_msgs::OccupancyGrid>(
            "/occupancy_grid", 10, std::bind(&OccupancyGridSubscriber::populate_cv_occupancy_grid, this, _1)
        )

        sensors_occupancy_grid_subscriber = this->create_subscription<nav_msgs::OccupancyGrid>(
            "/occupancy_grid", 10, std::bind(&OccupancyGridSubscriber::populate_sensors_occupancy_grid, this, _1)
        )

    private:
        // ------------- OCCUPANCY GRID VARIABLES -----------------

        // sensors occupancy grid variable
        OccupancyGrid sensors_occupancy_grid;

        // CV occupancy grid variable
        OccupancyGrid cv_occupancy_grid;

        // Combined Occupancy Grid variable
        OccupancyGrid global;
        

        // ------------------------ CALLBACKS -------------------------
        // populate cv occupancy grid
        void populate_cv_occupancy_grid(const OccupancyGrid &cv_cm)
        {
            //Maybe save snsr metadata once to increase efficency - resolution, width, height, origin
            cv_occupancy_grid.data = cv_cm->data;
            cv_occupancy_grid.info = cv_cm->info;
            cv_occupancy_grid.header = cv_cm->header;
             //cv_cm??
        }

        // populate global occupancy grid
        void populate_sensors_occupancy_grid(const OccupancyGrid &snsr_cm)
        {
            //Maybe save snsr metadata once to increase efficency - resolution, width, height, origin
            sensors_occupancy_grid.data = snsr_cm->data;
            sensors_occupancy_grid.info = snsr_cm->info;
            sensors_occupancy_grid.header = snsr_cm->header;
        }

        // //populate combined occupancy grid
        // void populate_combined_occupancy_grid(const OccupancyGrid &merge_cm)
        // {
        //     global.data = merge_cm->data;
        //     global.info = merge_cm->info;
        //     global.header = merge_cm->header;
        // }

        //populate global occupancy grid with values from cv
        void global_occupancy()
        {
            global = sensors_occupancy_grid;
            
            for(size_t i=0; i<=(size_t)cv_occupancy_grid.info.width; ++i)
            {
                for(size_t j=0; j<=(size_t)cv_occupancy_grid.info.height; ++j)
                {
                    global[i+(size_t)x][j+(size_t)y]=cv[i][j];
                }
            }
        }

        void publisher()
        {
            populate_cv_occupancy_grid(/*Subscribe to cv and put in here*/);
            populate_sensors_occupancy_grid(/*Subscribe to snsr and put in here*/);
            global_occupancy();
            global_occupancy_grid_publisher->publish(global); // TODO
        }
};

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MergeService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


// subscr to global OG, make a copy of it use that to init our OG
// writing the transform to get robot curr position ie. tf2
// overlay part - taking in curr position adn begin overlay

// You need to use tf2 to get the subscriber to get the robot to transform in the world frame
// 
