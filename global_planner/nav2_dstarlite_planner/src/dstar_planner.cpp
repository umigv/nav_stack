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
#include "nav2_util/node_utils.hpp"

#include "nav2_dstarlite_planner/dstar_planner.hpp"
#include "nav2_dstarlite_planner/Dstar.h"

namespace nav2_dstarlite_planner
{

void Dstar_planner::startup(){
    dstar = new Dstar(); 
     }
void Dstar_planner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  
  //std::cout << "config start" << std::endl;

  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  //std::cout << " costmap created\n";

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  startup();
    //std::cout << "config end" << std::endl;

}

void Dstar_planner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void Dstar_planner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());

}

void Dstar_planner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

nav_msgs::msg::Path Dstar_planner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  //std::cout << "createPlan init" << std::endl;
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

  global_path.header.stamp = node_->now();
 

    int pos_x = (int)((start.pose.position.x - costmap_->getOriginX()) / costmap_->getResolution());
    int pos_y = (int)((start.pose.position.y - costmap_->getOriginY()) / costmap_->getResolution());

    int goal_x = (int)((goal.pose.position.x - costmap_->getOriginX()) / costmap_->getResolution());
    int goal_y = (int)((goal.pose.position.y - costmap_->getOriginY()) / costmap_->getResolution());
  
  //std::cout << "planning" << std::endl;
  //std::cout << pos_x << std::endl;
  //std::cout << pos_y << std::endl;
  //std::cout << goal_x << std::endl;
  //std::cout << goal_y << std::endl;


  if(!dstar->gethasinit()){
    dstar->replan(); 
  }
  dstar->costMapCallback(costmap_);
  dstar->updateStart(pos_y, pos_x);
  dstar->updateGoal(goal_y, goal_x);
  global_path.header.frame_id = global_frame_;
  //if has init == false then run replan and costmap and upd and start
  // otherwise just do replan
  dstar->replan(); 

   
  
    list<state> mypath;
     mypath = dstar->getPath();     // retrieve path

    geometry_msgs::msg::PoseStamped pose;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.pose.position.z = 0.0;

    auto it = mypath.begin();
    while (it!=mypath.end())
    {
        
        float global_x = (it->x*costmap_->getResolution()) + costmap_->getOriginX();
        float global_y = (it->y*costmap_->getResolution()) + costmap_->getOriginY();  
        state u;
      //  dstar->printCell(it->x, it->y);
       // //std::cout << global_x << " " << global_y << std::endl ;
        pose.pose.position.x = global_y;
        pose.pose.position.y = global_x;
        pose.header.frame_id = "map";

        global_path.poses.push_back(pose);
        it++;
    }
    //std::cout << "createPlan end" << std::endl;

  return global_path;
}  // namespace nav2_straightline_planner

}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_dstarlite_planner::Dstar_planner, nav2_core::GlobalPlanner)