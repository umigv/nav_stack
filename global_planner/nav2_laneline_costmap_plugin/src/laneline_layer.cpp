// // /*********************************************************************
// //  *
// //  * Software License Agreement (BSD License)
// //  *
// //  *  Copyright (c) 2008, 2013, Willow Garage, Inc.
// //  *  Copyright (c) 2020, Samsung R&D Institute Russia
// //  *  All rights reserved.
// //  *
// //  *  Redistribution and use in source and binary forms, with or without
// //  *  modification, are permitted provided that the following conditions
// //  *  are met:
// //  *
// //  *   * Redistributions of source code must retain the above copyright
// //  *     notice, this list of conditions and the following disclaimer.
// //  *   * Redistributions in binary form must reproduce the above
// //  *     copyright notice, this list of conditions and the following
// //  *     disclaimer in the documentation and/or other materials provided
// //  *     with the distribution.
// //  *   * Neither the name of Willow Garage, Inc. nor the names of its
// //  *     contributors may be used to endorse or promote products derived
// //  *     from this software without specific prior written permission.
// //  *
// //  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// //  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// //  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// //  *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// //  *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// //  *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// //  *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// //  *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// //  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// //  *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// //  *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// //  *  POSSIBILITY OF SUCH DAMAGE.
// //  *
// //  * Author: Eitan Marder-Eppstein
// //  *         David V. Lu!!
// //  *         Alexey Merzlyakov
// //  *
// //  * Reference tutorial:
// //  * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
// //  *********************************************************************/
// // #include "nav2_laneline_costmap_plugin/laneline_layer.hpp"

// // #include "nav2_costmap_2d/costmap_math.hpp"
// // #include "nav2_costmap_2d/footprint.hpp"
// // #include "rclcpp/parameter_events_filter.hpp"

// // using nav2_costmap_2d::LETHAL_OBSTACLE;
// // using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
// // using nav2_costmap_2d::NO_INFORMATION;

// // namespace nav2_laneline_costmap_plugin
// // {

// // LanelineLayer::LanelineLayer()
// // : last_min_x_(-std::numeric_limits<float>::max()),
// //   last_min_y_(-std::numeric_limits<float>::max()),
// //   last_max_x_(std::numeric_limits<float>::max()),
// //   last_max_y_(std::numeric_limits<float>::max())
// // {
// // }

// // // This method is called at the end of plugin initialization.
// // // It contains ROS parameter(s) declaration and initialization
// // // of need_recalculation_ variable.
// // void
// // LanelineLayer::onInitialize()
// // {
// //   auto node = node_.lock(); 
// //   declareParameter("enabled", rclcpp::ParameterValue(true));
// //   node->get_parameter(name_ + "." + "enabled", enabled_);

// //   need_recalculation_ = false;
// //   current_ = true;
// //     cv_grid_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
// //       "/cv_grid", rclcpp::SystemDefaultsQoS(),
// //       std::bind(&LanelineLayer::cvGridCallback, this, std::placeholders::_1));
// // }

// // // The method is called to ask the plugin: which area of costmap it needs to update.
// // // Inside this method window bounds are re-calculated if need_recalculation_ is true
// // // and updated independently on its value.
// // void
// // LanelineLayer::updateBounds(
// //   double robot_x, double robot_y, double robot_yaw, double * min_x,
// //   double * min_y, double * max_x, double * max_y)
// // {
// //     if (need_recalculation_) {
// //     last_min_x_ = *min_x;
// //     last_min_y_ = *min_y;
// //     last_max_x_ = *max_x;
// //     last_max_y_ = *max_y;
// //     // For some reason when I make these -<double>::max() it does not
// //     // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
// //     // -<float>::max() instead.
// //     *min_x = -std::numeric_limits<float>::max();
// //     *min_y = -std::numeric_limits<float>::max();
// //     *max_x = std::numeric_limits<float>::max();
// //     *max_y = std::numeric_limits<float>::max();
// //     need_recalculation_ = false;
// //   } else {
// //     double tmp_min_x = last_min_x_;
// //     double tmp_min_y = last_min_y_;
// //     double tmp_max_x = last_max_x_;
// //     double tmp_max_y = last_max_y_;
// //     last_min_x_ = *min_x;
// //     last_min_y_ = *min_y;
// //     last_max_x_ = *max_x;
// //     last_max_y_ = *max_y;
// //     *min_x = std::min(tmp_min_x, *min_x);
// //     *min_y = std::min(tmp_min_y, *min_y);
// //     *max_x = std::max(tmp_max_x, *max_x);
// //     *max_y = std::max(tmp_max_y, *max_y);
// //   }

// //     // double cos_yaw = cos(robot_yaw);
// //     // double sin_yaw = sin(robot_yaw);
// //     //  cos_yaw =1;
// //     //  sin_yaw = 1;
// //     // // To ensure the bounds are always "in front" of the robot,
// //     // // we'll set the bounds based on 100 units behind (to the left in the robot's
// //     // // coordinate frame when facing toward positive yaw) up to 100 units in front
// //     // // (to the right in the robot's coordinate frame) and from robot's y up to 200 units in front.
// //     // *min_x = robot_x + (-100 * cos_yaw);
// //     // *min_y = robot_y + (0 * sin_yaw);
// //     // *max_x = robot_x + (100 * cos_yaw);
// //     // *max_y = robot_y + (200* sin_yaw);

// //     // // Optionally ensure the bounds are within the world coordinates
// //     // // if for example, the map should not update outside its current bounds.
// //     // double tmp_min_x = last_min_x_;
// //     // double tmp_min_y = last_min_y_;
// //     // double tmp_max_x = last_max_x_;
// //     // double tmp_max_y = last_max_y_;
// //     // *min_x = std::max(tmp_min_x, *min_x);
// //     // *min_y = std::max(tmp_min_y, *min_y);
// //     // *max_x = std::min(tmp_max_x, *max_x);
// //     // *max_y = std::min(tmp_max_y, *max_y);

    
// // //   if (need_recalculation_) {
// // //     last_min_x_ = *min_x;
// // //     last_min_y_ = *min_y;
// // //     last_max_x_ = *max_x;
// // //     last_max_y_ = *max_y;
// // //     // For some reason when I make these -<double>::max() it does not
// // //     // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
// // //     // -<float>::max() instead.
// // //     *min_x = -std::numeric_limits<float>::max();
// // //     *min_y = -std::numeric_limits<float>::max();
// // //     *max_x = std::numeric_limits<float>::max();
// // //     *max_y = std::numeric_limits<float>::max();
// // //     need_recalculation_ = false;
// // //   } else {
// // //     double tmp_min_x = last_min_x_;
// // //     double tmp_min_y = last_min_y_;
// // //     double tmp_max_x = last_max_x_;
// // //     double tmp_max_y = last_max_y_;
// // //     last_min_x_ = *min_x;
// // //     last_min_y_ = *min_y;
// // //     last_max_x_ = *max_x;
// // //     last_max_y_ = *max_y;
// // //     *min_x = std::min(tmp_min_x, *min_x);
// // //     *min_y = std::min(tmp_min_y, *min_y);
// // //     *max_x = std::max(tmp_max_x, *max_x);
// // //     *max_y = std::max(tmp_max_y, *max_y);
// // //   }
// // }

// // // The method is called when footprint was changed.
// // // Here it just resets need_recalculation_ variable.
// // void
// // LanelineLayer::onFootprintChanged()
// // {
// //   need_recalculation_ = true;

// //   RCLCPP_DEBUG(rclcpp::get_logger(
// //       "nav2_costmap_2d"), "LanelineLayer::onFootprintChanged(): num footprint points: %lu",
// //     layered_costmap_->getFootprint().size());
// // }

// // // The method is called when costmap recalculation is required.
// // // It updates the costmap within its window bounds.
// // // Inside this method the costmap gradient is generated and is writing directly
// // // to the resulting costmap master_grid without any merging with previous layers.
// // void
// // LanelineLayer::updateCosts(
// //   nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
// //   int max_i,
// //   int max_j)
// // {
// //         if (!enabled_ ) {
// //         return;
// //     }
// //     unsigned char * master_array = master_grid.getCharMap();
// //     // Assuming cv_grid_ is the received OccupancyGrid message and it's 200x200.
// //     int hmm = 0;
// //     for (int j = min_j; j < max_j; j++) {
// //         for (int i = min_i; i < max_i; i++) {
// //         // Convert the coordinates to the OccupancyGrid coordinate system as needed
// //         // ... Transform i, j to cv_grid_ coordinates here ...

// //         int index = master_grid.getIndex(i, j);
// //         int cv_data = cv_grid_.data[hmm];
// //         hmm++;

// //         // Update master_grid based on cv_grid value
// //         if (cv_data > 50) {
// //              master_array[index] = 250;
// //         } else {
// //              master_array[index] = 250; //TODO change
// //         }
// //         }
// //     }

// //     std::cout<<"stuff stuff stuff"<<min_i << " " << min_j << " " << max_i << " " << max_j << std::endl;


// // //   if (!enabled_) {
// // //     return;
// // //   }

// // //   // master_array - is a direct pointer to the resulting master_grid.
// // //   // master_grid - is a resulting costmap combined from all layers.
// // //   // By using this pointer all layers will be overwritten!
// // //   // To work with costmap layer and merge it with other costmap layers,
// // //   // please use costmap_ pointer instead (this is pointer to current
// // //   // costmap layer grid) and then call one of updates methods:
// // //   // - updateWithAddition()
// // //   // - updateWithMax()
// // //   // - updateWithOverwrite()
// // //   // - updateWithTrueOverwrite()
// // //   // In this case using master_array pointer is equal to modifying local costmap_
// // //   // pointer and then calling updateWithTrueOverwrite():
// // //   unsigned char * master_array = master_grid.getCharMap();
// // //   unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

// // //   // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
// // //   // These variables are used to update the costmap only within this window
// // //   // avoiding the updates of whole area.
// // //   //
// // //   // Fixing window coordinates with map size if necessary.
// // //   min_i = std::max(0, min_i);
// // //   min_j = std::max(0, min_j);
// // //   max_i = std::min(static_cast<int>(size_x), max_i);
// // //   max_j = std::min(static_cast<int>(size_y), max_j);

// // //   // Simply computing one-by-one cost per each cell
// // //   int gradient_index;
// // //   for (int j = min_j; j < max_j; j++) {
// // //     // Reset gradient_index each time when reaching the end of re-calculated window
// // //     // by OY axis.
// // //     gradient_index = 0;
// // //     for (int i = min_i; i < max_i; i++) {
// // //       int index = master_grid.getIndex(i, j);
// // //       // setting the gradient cost
// // //       unsigned char cost = (LETHAL_OBSTACLE - gradient_index*GRADIENT_FACTOR)%255;
// // //       if (gradient_index <= GRADIENT_SIZE) {
// // //         gradient_index++;
// // //       } else {
// // //         gradient_index = 0;
// // //       }
// // //       master_array[index] = cost;
// // //     }
// // //   }
// // }


// // void LanelineLayer::cvGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
// //   {
// //     // Store the grid message in a member variable.
// //     cv_grid_ = *msg;
// //     // Signal that we have received a new grid and need to update the costs.
// //   }

 
// //   //...

// // }  // namespace nav2_laneline_costmap_plugin

// // // This is the macro allowing a nav2_laneline_costmap_plugin::LanelineLayer class
// // // to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// // // Usually places in the end of cpp-file where the loadable class written.
// // #include "pluginlib/class_list_macros.hpp"
// // PLUGINLIB_EXPORT_CLASS(nav2_laneline_costmap_plugin::LanelineLayer, nav2_costmap_2d::Layer)

// /*********************************************************************
//  *
//  * Software License Agreement (BSD License)
//  *
//  *  Copyright (c) 2008, 2013, Willow Garage, Inc.
//  *  Copyright (c) 2020, Samsung R&D Institute Russia
//  *  All rights reserved.
//  *
//  *  Redistribution and use in source and binary forms, with or without
//  *  modification, are permitted provided that the following conditions
//  *  are met:
//  *
//  *   * Redistributions of source code must retain the above copyright
//  *     notice, this list of conditions and the following disclaimer.
//  *   * Redistributions in binary form must reproduce the above
//  *     copyright notice, this list of conditions and the following
//  *     disclaimer in the documentation and/or other materials provided
//  *     with the distribution.
//  *   * Neither the name of Willow Garage, Inc. nor the names of its
//  *     contributors may be used to endorse or promote products derived
//  *     from this software without specific prior written permission.
//  *
//  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  *  POSSIBILITY OF SUCH DAMAGE.
//  *
//  * Author: Eitan Marder-Eppstein
//  *         David V. Lu!!
//  *         Alexey Merzlyakov
//  *
//  * Reference tutorial:
//  * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
//  *********************************************************************/
// #include "nav2_laneline_costmap_plugin/laneline_layer.hpp"

// #include "nav2_costmap_2d/costmap_math.hpp"
// #include "nav2_costmap_2d/footprint.hpp"
// #include "rclcpp/parameter_events_filter.hpp"

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
//        "/cv_grid", rclcpp::SystemDefaultsQoS(),
//        std::bind(&LanelineLayer::cvGridCallback, this, std::placeholders::_1));
// }

// // The method is called to ask the plugin: which area of costmap it needs to update.
// // Inside this method window bounds are re-calculated if need_recalculation_ is true
// // and updated independently on its value.
// void
// LanelineLayer::updateBounds(
//   double robot_x, double robot_y, double /*robot_yaw*/, double * min_x,
//   double * min_y, double * max_x, double * max_y)
// {
//      std::cout<<"stuff stuff stuff"<<robot_x << " " << robot_y << std::endl;

//   if (need_recalculation_) {
//     last_min_x_ = *min_x;
//     last_min_y_ = *min_y;
//     last_max_x_ = *max_x;
//     last_max_y_ = *max_y;
//     // For some reason when I make these -<double>::max() it does not
//     // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
//     // -<float>::max() instead.
//     *min_x = -std::numeric_limits<float>::max();
//     *min_y = -std::numeric_limits<float>::max();
//     *max_x = std::numeric_limits<float>::max();
//     *max_y = std::numeric_limits<float>::max();
//     // *min_x = 0 - 150.0;
//     // *min_y = 0; 
//     // *max_x = 0 + 150.0;
//     // *max_y = 0 + 300.0;

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
//   }

//    std::cout<<"stuff stuff stuff"<<min_x << " " << min_y << " " << max_x << " " << max_y << std::endl;

// }

// // The method is called when footprint was changed.
// // Here it just resets need_recalculation_ variable.
// void
// LanelineLayer::onFootprintChanged()
// {
//   need_recalculation_ = true;

//   RCLCPP_DEBUG(rclcpp::get_logger(
//       "nav2_costmap_2d"), "LanelineLayer::onFootprintChanged(): num footprint points: %lu",
//     layered_costmap_->getFootprint().size());
// }

// // void LanelineLayer::cvGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
// //   {
// //     // Store the grid message in a member variable.
// //     cv_grid_ = *msg;
// //     // Signal that we have received a new grid and need to update the costs.
// //   }


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
//   if (!enabled_) {
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

//   // Simply computing one-by-one cost per each cell
//   int gradient_index;
//   for (int j = min_j; j < max_j; j++) {
//     // Reset gradient_index each time when reaching the end of re-calculated window
//     // by OY axis.
//     gradient_index = 0;
//     for (int i = min_i; i < max_i; i++) {
//       int index = master_grid.getIndex(i, j);
//       // setting the gradient cost
//       unsigned char cost = (LETHAL_OBSTACLE - gradient_index*GRADIENT_FACTOR)%255;
//       if (gradient_index <= GRADIENT_SIZE) {
//         gradient_index++;
//       } else {
//         gradient_index = 0;
//       }
//       master_array[index] = cost;
//     }
//   }
// }

// }  // namespace nav2_laneline_costmap_plugin

// // This is the macro allowing a nav2_laneline_costmap_plugin::LanelineLayer class
// // to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// // Usually places in the end of cpp-file where the loadable class written.
// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(nav2_laneline_costmap_plugin::LanelineLayer, nav2_costmap_2d::Layer)

#include "nav2_laneline_costmap_plugin/laneline_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include <fstream>


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
  auto node = node_.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  need_recalculation_ = false;
  current_ = true;
      cv_grid_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/cv_grid", rclcpp::SystemDefaultsQoS(),
      std::bind(&LanelineLayer::cvGridCallback, this, std::placeholders::_1));

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
  double robot_x, double robot_y, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
     std::cout<<"robot robot robot"<<robot_x << " " << robot_y << std::endl;

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
  need_recalculation_ = true;
  std::cout << " NEEEEEEDDDDDDR RRRRRRREEEEEEEEEE CCCAAAAAAAAAALLLLLLLLCCCCCCC" << std::endl;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "LanelineLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
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
  int gradient_index;
  int count  = 0;

    
   std::ofstream ofs;
    ofs.open("mapper.txt", std::ofstream::out | std::ofstream::trunc);
    ofs.close();

    std::ofstream myfile;
    myfile.open ("mapper.txt");

    std::cout << "OVER HERER" << std::endl;


  for (int j = min_j; j < max_j; j++) {
    // Reset gradient_index each time when reaching the end of re-calculated window
    // by OY axis.
    gradient_index = 0;
    for (int i = min_i; i < max_i; i++) {
      int index = master_grid.getIndex(i, j);
      // setting the gradient cost
      unsigned char cost = (LETHAL_OBSTACLE - gradient_index*GRADIENT_FACTOR)%255;
      int8_t data_point =0;
      // std:: cout << " data point " << data_point << std::endl;
      count = (j - min_j) * 300 + (i-min_i); 
      if (count < 300*300)
         data_point = cv_grid_.data[count];

         
      // std::cout<< "AHHHHHH OUT OF GRID " << count << std::endl;
      myfile << data_point << " ";
      //count++;
      if (data_point >= 50) {
         cost = LETHAL_OBSTACLE;
          master_array[index] = cost;
      } else {
        cost = 0;
      }

     
    }
    myfile << "\n";
  }
}

}  // namespace nav2_laneline_costmap_plugin

// This is the macro allowing a nav2_laneline_costmap_plugin::LanelineLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_laneline_costmap_plugin::LanelineLayer, nav2_costmap_2d::Layer)