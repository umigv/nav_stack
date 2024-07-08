
#include "nav2_laneline_costmap_plugin/laneline_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include <fstream>
#include <cmath>

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_laneline_costmap_plugin
{

LanelineLayer::LanelineLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void
LanelineLayer::onInitialize()
{
  std::cout << "start init" <<std::endl;
  auto node = node_.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);
  
  need_recalculation_ = false;
  current_ = true;
      cv_grid_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/cv_view", rclcpp::SystemDefaultsQoS(),
      std::bind(&LanelineLayer::cvGridCallback, this, std::placeholders::_1));
    std::cout << "end init" << std::endl;
}



void LanelineLayer::cvGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    // Store the grid message in a member variable.
    cv_grid_ = *msg;
    // Signal that we have received a new grid and need to update the costs.
  }

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void
LanelineLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
     std::cout<<"robot robot robot"<<robot_x << " " << robot_y << std::endl;
   yaw = robot_yaw;
  if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.

    *min_x = robot_x * 1.0  - 7.50;
    *min_y = robot_y * 1.0  - 7.50;
    *max_x =  robot_x * 1.0  + 7.5;
    *max_y = robot_y * 1.0  + 7.50;
         std::cout<<"real real real"<<*min_x << " " <<* min_y << " " << *max_x << " " << *max_y << std::endl;

    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
       
    *min_x = robot_x * 1.0  - 7.50;
    *min_y = robot_y * 1.0  - 7.50;
    *max_x =  robot_x * 1.0  + 7.5;
    *max_y = robot_y * 1.0  + 7.50;
  }
     std::cout<<"stuff stuff stuff"<<*min_x << " " <<* min_y << " " << *max_x << " " << *max_y << std::endl;

}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void
LanelineLayer::onFootprintChanged()
{
  std::cout << " NEEEEEEDDDDDDR RRRRRRREEEEEEEEEE CCCAAAAAAAAAALLLLLLLLCCCCCCC" << std::endl;
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "LanelineLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
      std::cout << " footpring end" << std::endl;

}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void
LanelineLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  std::cout << " updating " << min_i << " " << min_j << " " << max_i << " " << max_j << std::endl; 
  if (!enabled_) {
      std::cout << " NOT ENDABLE " << std::endl; 

    return;
  }
  if(min_i == 0 && max_i == 100){
    std::cout << " ahhah " << std::endl; 
    return;
  }

  // master_array - is a direct pointer to the resulting master_grid.
  // master_grid - is a resulting costmap combined from all layers.
  // By using this pointer all layers will be overwritten!
  // To work with costmap layer and merge it with other costmap layers,
  // please use costmap_ pointer instead (this is pointer to current
  // costmap layer grid) and then call one of updates methods:
  // - updateWithAddition()
  // - updateWithMax()
  // - updateWithOverwrite()
  // - updateWithTrueOverwrite()
  // In this case using master_array pointer is equal to modifying local costmap_
  // pointer and then calling updateWithTrueOverwrite():
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
  // These variables are used to update the costmap only within this window
  // avoiding the updates of whole area.
  //
  // Fixing window coordinates with map size if necessary.
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);
  std::cout << " fixing " << min_i << " " << min_j << " " << max_i << " " << max_j << std::endl; 

  // Simply computing one-by-one cost per each cell
  // int data_index = i * cv_grid_.info.width + j;
  int count  = 0;


   std::ofstream ofs;
    ofs.open("mapper.txt", std::ofstream::out | std::ofstream::trunc);
    ofs.close();

    std::ofstream myfile;
    myfile.open ("mapper.txt");


  std::cout << "OVER HERER" << std::endl;
  int H = 155;
  int W = 76;
  int robot_x = (max_i - min_i) /2 + min_i; //TODO bit shift 
  int robot_y = (max_j - min_j) /2 + min_j;
        std::cout << "cen" <<  + robot_x << " " << robot_y << std::endl;
  int counter = 0 ;
  for (int j = 0 ; j < W; j++){
    for( int i = H-1 ; i >= 0; i --){
      int J = j ;//- (.5*W);
      int I = i - (.5*H);
      int new_x = ((J) * cos(yaw) ) -  (I*sin(yaw));
      int new_y = ((J) * sin(yaw) ) + (I*cos(yaw));
      // int find_x = new_x + robot_x;
      // int find_y = new_y + robot_y;
      if(robot_x + new_x  <  min_i || robot_x + new_x > max_i ){
          std::cout << "out of max bounds" << new_x + robot_x << " " << new_y + robot_y << std::endl;


      }
       if(robot_y + new_y  <  min_j || robot_y + new_y > max_j ){
          std::cout << "out of max bounds" << new_x + robot_x << " " << new_y + robot_y << std::endl;

      }
      int index = master_grid.getIndex(robot_x + new_x , robot_y+new_y );
     // std::cout << "neq" << new_x + robot_x << " " << new_y + robot_y << std::endl;
      //std::cout << " size " << size_x << " " << size_y << " " << robot_x + new_x << " " << robot_y+new_y << std::endl; 

      int8_t data_point = 0;
      count = (j) * 76 + (i); 
      unsigned char cost = 0;
      // if(){
      //   std:: cout << "WWHOOOA OUT OF BOUNDS" << std::endl;
      // }
      if ((int)master_grid.getSizeInCellsX() * (int)master_grid.getSizeInCellsY() > index && count < 155*76){
        data_point = cv_grid_.data[count];
      

      }else{
       std::cout << "view bounds out of " << std::endl;
        data_point = -2;
      }
     // std::cout << "data point" << (int)data_point;
      data_point =-1;

      if ((int)data_point == -1) {
         cost = LETHAL_OBSTACLE;
         cost = counter;
         counter +=1;
         if (counter > 11779  ){
          counter = 11779;
         }
          cost = cv_grid_.data[counter];
          myfile <<  std::setfill('0') << std::setw(3) << (int) cost << " ";
          master_array[index] = cost;
      } else {
        cost = 0;
      }
    }
     myfile << "\n ";
  }
 std::cout << "end 1" << std::endl;
   //center block for robot
  // for(int i = 0; i < 15; i ++){
  //   for(int j = 0 ; j < 30; j++){
  //     unsigned char cost = 0; 
  //     int index = master_grid.getIndex (robot_x + i - 7, robot_y + j -15 );
  //     cost = LETHAL_OBSTACLE;
  //     master_array[index] = cost;
  //   }
  // }


 std::cout << "end 2" << std::endl;


  // int gradient_index = 0;
  // for (int j = min_j; j < max_j; j++) {
  //   // Reset gradient_index each time when reaching the end of re-calculated window
  //   // by OY axis.
  //   gradient_index = 0;
  //   for (int i = min_i; i < max_i; i++) {

  //     int index = master_grid.getIndex(i, j);
  //     // setting the gradient cost
  //     unsigned char cost = (LETHAL_OBSTACLE - gradient_index*GRADIENT_FACTOR)%255;
  //     int8_t data_point =0;
  //     std:: cout << " data point " << i << " " << j  << std::endl;
  //     count = (j - min_j) * 155 + (i-min_i); 
  //     if (count < 155*76)
  //        data_point = cv_grid_.data[count];

         
  //     // std::cout<< "AHHHHHH OUT OF GRID " << count << std::endl;
  //     myfile << data_point << " ";
  //     //count++;
  //     data_point = 50 ; 
  //     if (data_point >= 50) {
  //        cost = LETHAL_OBSTACLE;
  //         master_array[index] = cost;
  //     } else {
  //       cost = 0;
  //     }

     
  //   }
  //   myfile << "\n";
  // }
}

}  // namespace nav2_laneline_costmap_plugin

// This is the macro allowing a nav2_laneline_costmap_plugin::LanelineLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_laneline_costmap_plugin::LanelineLayer, nav2_costmap_2d::Layer)


// This is the macro allowing a nav2_laneline_costmap_plugin::LanelineLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(nav2_laneline_costmap_plugin::LanelineLayer, nav2_costmap_2d::Layer)

// #include "nav2_laneline_costmap_plugin/laneline_layer.hpp"

// #include "nav2_costmap_2d/costmap_math.hpp"
// #include "nav2_costmap_2d/footprint.hpp"
// #include "rclcpp/parameter_events_filter.hpp"
// #include <fstream>


// using nav2_costmap_2d::LETHAL_OBSTACLE;
// using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
// using nav2_costmap_2d::NO_INFORMATION;

// namespace nav2_laneline_costmap_plugin
// {

// LanelineLayer::LanelineLayer()
// : last_min_x_(-std::numeric_limits<float>::max()),
//   last_min_y_(-std::numeric_limits<float>::max()),
//   last_max_x_(std::numeric_limits<float>::max()),
//   last_max_y_(std::numeric_limits<float>::max())
// {
// }

// // This method is called at the end of plugin initialization.
// // It contains ROS parameter(s) declaration and initialization
// // of need_recalculation_ variable.
// void
// LanelineLayer::onInitialize()
// {
//   auto node = node_.lock(); 
//   declareParameter("enabled", rclcpp::ParameterValue(true));
//   node->get_parameter(name_ + "." + "enabled", enabled_);

//   need_recalculation_ = false;
//   current_ = true;
//       cv_grid_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
//       "/cv_grid", rclcpp::SystemDefaultsQoS(),
//       std::bind(&LanelineLayer::cvGridCallback, this, std::placeholders::_1));

// }



// void LanelineLayer::cvGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
//   {
//     // Store the grid message in a member variable.
//     cv_grid_ = *msg;
//     // Signal that we have received a new grid and need to update the costs.
//   }

// // The method is called to ask the plugin: which area of costmap it needs to update.
// // Inside this method window bounds are re-calculated if need_recalculation_ is true
// // and updated independently on its value.
// void
// LanelineLayer::updateBounds(
//   double robot_x, double robot_y, double robot_yaw, double * min_x,
//   double * min_y, double * max_x, double * max_y)
// {
//      std::cout<<"robot robot robot"<<robot_x << " " << robot_y << std::endl;

//   if (need_recalculation_) {
//     last_min_x_ = *min_x;
//     last_min_y_ = *min_y;
//     last_max_x_ = *max_x;
//     last_max_y_ = *max_y;
//     // For some reason when I make these -<double>::max() it does not
//     // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
//     // -<float>::max() instead.

//     *min_x = robot_x * 1.0  - 7.50;
//     *min_y = robot_y * 1.0  - 7.50;
//     *max_x =  robot_x * 1.0  + 7.5;
//     *max_y = robot_y * 1.0  + 7.50;
//          std::cout<<"real real real"<<*min_x << " " <<* min_y << " " << *max_x << " " << *max_y << std::endl;

//     need_recalculation_ = false;
//   } else {
//     double tmp_min_x = last_min_x_;
//     double tmp_min_y = last_min_y_;
//     double tmp_max_x = last_max_x_;
//     double tmp_max_y = last_max_y_;
//     last_min_x_ = *min_x;
//     last_min_y_ = *min_y;
//     last_max_x_ = *max_x;
//     last_max_y_ = *max_y;
//     *min_x = std::min(tmp_min_x, *min_x);
//     *min_y = std::min(tmp_min_y, *min_y);
//     *max_x = std::max(tmp_max_x, *max_x);
//     *max_y = std::max(tmp_max_y, *max_y);
       
//     *min_x = robot_x * 1.0  - 7.50;
//     *min_y = robot_y * 1.0  - 7.50;
//     *max_x =  robot_x * 1.0  + 7.5;
//     *max_y = robot_y * 1.0  + 7.50;
//   }
//      std::cout<<"stuff stuff stuff"<<*min_x << " " <<* min_y << " " << *max_x << " " << *max_y << std::endl;

// }

// // The method is called when footprint was changed.
// // Here it just resets need_recalculation_ variable.
// void
// LanelineLayer::onFootprintChanged()
// {
//   need_recalculation_ = true;
//   std::cout << " NEEEEEEDDDDDDR RRRRRRREEEEEEEEEE CCCAAAAAAAAAALLLLLLLLCCCCCCC" << std::endl;

//   RCLCPP_DEBUG(rclcpp::get_logger(
//       "nav2_costmap_2d"), "LanelineLayer::onFootprintChanged(): num footprint points: %lu",
//     layered_costmap_->getFootprint().size());
// }

// // The method is called when costmap recalculation is required.
// // It updates the costmap within its window bounds.
// // Inside this method the costmap gradient is generated and is writing directly
// // to the resulting costmap master_grid without any merging with previous layers.
// void
// LanelineLayer::updateCosts(
//   nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
//   int max_i,
//   int max_j)
// {
//   std::cout << " updating " << min_i << " " << min_j << " " << max_i << " " << max_j << std::endl; 
//   if (!enabled_) {
//       std::cout << " NOT ENDABLE " << std::endl; 

//     return;
//   }

//   // master_array - is a direct pointer to the resulting master_grid.
//   // master_grid - is a resulting costmap combined from all layers.
//   // By using this pointer all layers will be overwritten!
//   // To work with costmap layer and merge it with other costmap layers,
//   // please use costmap_ pointer instead (this is pointer to current
//   // costmap layer grid) and then call one of updates methods:
//   // - updateWithAddition()
//   // - updateWithMax()
//   // - updateWithOverwrite()
//   // - updateWithTrueOverwrite()
//   // In this case using master_array pointer is equal to modifying local costmap_
//   // pointer and then calling updateWithTrueOverwrite():
//   unsigned char * master_array = master_grid.getCharMap();
//   unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

//   // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
//   // These variables are used to update the costmap only within this window
//   // avoiding the updates of whole area.
//   //
//   // Fixing window coordinates with map size if necessary.
//   min_i = std::max(0, min_i);
//   min_j = std::max(0, min_j);
//   max_i = std::min(static_cast<int>(size_x), max_i);
//   max_j = std::min(static_cast<int>(size_y), max_j);
//   std::cout << " fixing " << min_i << " " << min_j << " " << max_i << " " << max_j << std::endl; 

//   // Simply computing one-by-one cost per each cell
//   // int data_index = i * cv_grid_.info.width + j;
//   int gradient_index;
//   int count  = 0;

    
//    std::ofstream ofs;
//     ofs.open("mapper.txt", std::ofstream::out | std::ofstream::trunc);
//     ofs.close();

//     std::ofstream myfile;
//     myfile.open ("mapper.txt");

//     std::cout << "OVER HERER" << std::endl;


//   for (int j = min_j; j < max_j; j++) {
//     // Reset gradient_index each time when reaching the end of re-calculated window
//     // by OY axis.
//     gradient_index = 0;
//     for (int i = min_i; i < max_i; i++) {
//       int index = master_grid.getIndex(i, j);
//       // setting the gradient cost
//       unsigned char cost = (LETHAL_OBSTACLE - gradient_index*GRADIENT_FACTOR)%255;
//       int8_t data_point =0;
//       // std:: cout << " data point " << data_point << std::endl;
//       count = (j - min_j) * 300 + (i-min_i); 
//       if (count < 300*300)
//          data_point = cv_grid_.data[count];

         
//       // std::cout<< "AHHHHHH OUT OF GRID " << count << std::endl;
//       myfile << data_point << " ";
//       //count++;
//       if (data_point >= 50) {
//          cost = LETHAL_OBSTACLE;
//           master_array[index] = cost;
//       } else {
//         cost = 0;
//       }

     
//     }
//     myfile << "\n";
//   }
// }

// }  // namespace nav2_laneline_costmap_plugin

// // This is the macro allowing a nav2_laneline_costmap_plugin::LanelineLayer class
// // to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// // Usually places in the end of cpp-file where the loadable class written.
// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(nav2_laneline_costmap_plugin::LanelineLayer, nav2_costmap_2d::Layer)