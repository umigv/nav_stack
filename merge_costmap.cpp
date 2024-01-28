#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using std::placeholders::_1;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

// Subsribes to the global occupancy grid and the cv occupancy grid 
// and merges them together to create a new global occupancy grid
// that is then published to the global costmap
class MergeService : public rclcpp::Node {
    public:
        MergeService() : Node("merge_service") {
        cv_og_subsriber_ = this->create_subscription<OccupancyGrid>(
            "/occupancy_grid", 10, std::bind(&MergeService::populate_cv_occupancy_grid, this, _1)
        )

        sensors_occupancy_grid_subsriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/occupancy_grid", 10, std::bind(&MergeService::populate_sensors_occupancy_grid, this, _1)
        )
    }


    private:
        // ------------- OCCUPANCY GRID VARIABLES -----------------

        // sensors occupancy grid variable
        OccupancyGrid sensors_occupancy_grid;

        // CV occupancy grid variable
        OccupancyGrid cv_occupancy_grid;

        // Combined Occupancy Grid variable
        OccupancyGrid combined_occupancy_grid;
        

        // ------------------------CALLBACKS -------------------------
        // populate cv occupancy grid
        void populate_cv_occupancy_grid(const OccupancyGrid &cv_cm)
        {
            //Maybe save snsr metadata once to increase efficency - resolution, width, height, origin
            cv_occupancy_grid.data = cv_cm->data;
            cv_occupancy_grid.info = cv_cm->info;
            cv_occupancy_grid.header = cv_cm->header; //cv_cm??
        }

        // populate global occupancy grid
        void populate_sensors_occupancy_grid(const OccupancyGrid &snsr_cm)
        {
            //Maybe save snsr metadata once to increase efficency - resolution, width, height, origin
            sensors_occupancy_grid.data = snsr_cm->data;
            sensors_occupancy_grid.info = snsr_cm->info;
            sensors_occupancy_grid.header = snsr_cm->header;
        }

        //populate combined occupancy grid
        void populate_combined_occupancy_grid(const OccupancyGrid &cv_cm, const OccupancyGrid &snsr_cm)
        {
            //Tf2 local data into global with gps
            combined_occupancy_grid.info = snsr_cm->info;
            combined_occupancy_grid.header = snsr_cm->header;
        }

        void tf2_local_to_global()
        {
            
        }

        OccupancyGrid &merge_occupancy_grids()
        {

        }

}


// subscr to global OG, make a copy of it use that to init our OG
// writing the transform to get robot curr position ie. tf2
// overlay part - taking in curr position adn begin overlay