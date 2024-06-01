// Copyright 2021 Jaehyun Shim
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

int main(int argc, char * argv[])
{

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr cancel_navto_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node,"/navigate_to_pose");

    cancel_navto_client->async_cancel_all_goals();
}