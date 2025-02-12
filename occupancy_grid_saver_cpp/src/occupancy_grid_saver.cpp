#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <fstream>
#include <vector>
#include <unordered_set>
#include <set>


class OccupancyGridSaver : public rclcpp::Node
{
public:
    OccupancyGridSaver()
        : Node("occupancy_grid_saver"), saved(false)
    {
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/inflated_occ", 10,
            std::bind(&OccupancyGridSaver::occupancy_grid_callback, this, std::placeholders::_1));
    }

    // double OccupancyGridSaver::get_angle_difference(double angle1, double angle2)
    // {
    //     double diff = fmod(angle1 - angle2, 2 * M_PI);
    //     if (diff > M_PI)
    //     {
    //         diff -= 2 * M_PI;
    //     }
    //     else if (diff < -M_PI)
    //     {
    //         diff += 2 * M_PI;
    //     }
    //     return diff;
    // }


private:

    void goal_finder(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        // if (position.first == 0 && position.second == 0)  // Position is None
        // {
        //     return;
        // }
        
        
        // std::pair<int, int> robot_pose = {23,77}
        // std::vector<int8_t> grid_data = msg->data;
        // std::pair<int, int> temp_best_pos = robot_pose; //TODO ask CV is this right
        // double best_pos_cost = -1000;

        // std::set<std::pair<int, int>> frontier;
	    // frontier.insert(robot_pose);
	    // std::unordered_set<int> visted;


        return ;




    }


    void occupancy_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if (!saved)
        {
            saved = true;
            RCLCPP_INFO(this->get_logger(), "Received an OccupancyGrid message. Saving data...");

            // Save data to a file
            std::ofstream file("inflated_pic_occupancy_grid_data.txt");
            if (file.is_open())
            {
                file << "Width: " << msg->info.width << ", Height: " << msg->info.height << "\n";
                for (size_t i = 0; i < msg->data.size(); ++i)
                {
                    file << (int)msg->data[i] << " ";
                }
                file << "\n";
                file.close();
            }
            RCLCPP_INFO(this->get_logger(), "Data saved to occupancy_grid_data.txt");

            int width = msg->info.width;
            int height = msg->info.height;
            std::vector<int8_t> data = msg->data;


            
            RCLCPP_INFO(this->get_logger(), "Calling the goal finder");

            RCLCPP_INFO(this->get_logger(), "Shutting down node after saving and displaying the data.");
            rclcpp::shutdown();

        }
    }
    

    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    bool saved;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyGridSaver>());
    rclcpp::shutdown();
    return 0;
}
