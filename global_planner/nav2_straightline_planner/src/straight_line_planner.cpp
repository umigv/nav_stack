/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#include <cmath>
#include <string>
#include <memory>
#include <iostream>

#include "nav2_util/node_utils.hpp"

#include "nav2_straightline_planner/straight_line_planner.hpp"
#include "nav2_straightline_planner/rpastar.h"


namespace nav2_straightline_planner
{

void StraightLine::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{

  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  std::cout << " costmap created\n";
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void StraightLine::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type RPASTARLine",
    name_.c_str());
}

void StraightLine::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type RPASTARLine",
    name_.c_str());
}

void StraightLine::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type RPASTARLine",
    name_.c_str());
}

nav_msgs::msg::Path StraightLine::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;
  std::vector<geometry_msgs::msg::PoseStamped> plan;

  
   if(!makePlan(start, goal, 0.5, plan)){
      // throw nav2_core::NoValidPathCouldBeFound(
      //       "Failed to create plan with tolerance of: " + std::to_string(0.0) );
      std::cout<<"Failed to create a plan\n";
   }
  for(auto x : plan){
    global_path.poses.push_back(x);
  }

  return global_path;

}
bool StraightLine::makePlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal,  double tolerance, std::vector<geometry_msgs::msg::PoseStamped>& plan){

    RCLCPP_INFO(
     node_->get_logger(), "Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

   
    
    int pos_x = (int)((start.pose.position.x - costmap_->getOriginX()) / costmap_->getResolution());
    int pos_y = (int)((start.pose.position.y - costmap_->getOriginY()) / costmap_->getResolution());

    int goal_x = (int)((goal.pose.position.x - costmap_->getOriginX()) / costmap_->getResolution());
    int goal_y = (int)((goal.pose.position.y - costmap_->getOriginY()) / costmap_->getResolution());
    // for (int i = 0; i < map.info.height; i++)
    // {
    //   for (int j = 0; j < map.info.width; j++)
    //   {
    //     std::cout << map.data[map.info.width*i + j] << "  ";
    //   }
    //   std::cout << std::endl;
    // }
    std::cout << pos_x << " " << pos_y << std::endl;
    std::cout << goal_x << " " << goal_y << std::endl;
    std::cout << "Running A*..." << std::endl << std::endl;
    //WHY SWITCH THESE GUYS???
    std::pair<int,int> first(pos_y, pos_x);
    std::pair<int,int> last(goal_y, goal_x);
        std::cout << "Creating runner ..." << std::endl << std::endl;
    //rpastar runner = rpastar(first, last, costmap_);

    rpastar runner(first, last, costmap_);

    std::cout << "Searching..." << std::endl << std::endl;

    runner.search();
    if (runner.goal_found())
    {
        std::vector<std::pair<int,int>> path = runner.backtracker();
        std::cout << "Path found!" << std::endl;
        generate_path(costmap_,path,plan);
    }
    std::cout << "done with make plan\n";

    return true;
}

void StraightLine::generate_path(const nav2_costmap_2d::Costmap2D* map, std::vector<std::pair<int,int>> &path, std::vector<geometry_msgs::msg::PoseStamped>& plan)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.orientation.w = 0.5;
    pose.pose.orientation.x = 0.5;
    pose.pose.orientation.y = 0.5;
    pose.pose.orientation.z = 0.5;

    for (int i = 0; i < path.size(); i++)
    {
        float global_x = (path[i].first*map->getResolution()) + map->getOriginX();
        float global_y = (path[i].second*map->getResolution()) + map->getOriginY();
        pose.pose.position.x = global_y;
        pose.pose.position.y = global_x;
        pose.header.frame_id = "map";
        plan.push_back(pose);
    }

    return;
}



}  // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)
