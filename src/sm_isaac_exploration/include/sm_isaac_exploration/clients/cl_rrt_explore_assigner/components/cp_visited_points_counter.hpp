// Copyright 2021 RobosoftAI Inc.
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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <string>

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/component.hpp>
#include <std_msgs/msg/string.hpp>
// namespace sm_isaac_exploration 
// {
namespace cl_rrt_explore_assigner
{
struct EvExplorationFinished : sc::event<EvExplorationFinished> {};

// this class is used to switch the current goal checker of the remote navigation2 stack controller
class CpVisitedPointsCounter : public smacc2::ISmaccComponent
{
public:
  CpVisitedPointsCounter();
  void onInitialize() override;
  virtual ~CpVisitedPointsCounter();
  void countVisitedPoint();
  int getVisitedPoints();
  int getMaximumVisitedPoints();

private:
//   std::string goal_checker_selector_topic_;
//   std::string default_goal_checker_name_;
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_checker_selector_pub_;
    int visited_points_counter_;
    int max_visit_;
};
}  // namespace cl_rrt_explore_assigner
// }  // namespace sm_isaac_exploration