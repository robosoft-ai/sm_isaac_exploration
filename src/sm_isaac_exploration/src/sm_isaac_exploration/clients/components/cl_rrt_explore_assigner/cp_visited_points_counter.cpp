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
#include <sm_isaac_exploration/clients/cl_rrt_explore_assigner/components/cp_visited_points_counter.hpp>

// namespace sm_isaac_exploration 
// {

namespace cl_rrt_explore_assigner
{
CpVisitedPointsCounter::CpVisitedPointsCounter()
{
}

CpVisitedPointsCounter::~CpVisitedPointsCounter() {}

void CpVisitedPointsCounter::onInitialize()
{
    visited_points_counter_ = 0;
    max_visit_ = 0;

    auto nh = getNode();
    nh->declare_parameter("max_visited_points", this->max_visit_);

    if(nh->get_parameter("max_visited_points", max_visit_)){
      RCLCPP_FATAL(getLogger(), "[CpVisitedPointsCounter] MAX POINTS TO VISIT: %d", max_visit_);
    }
   
//   rclcpp::QoS qos(rclcpp::KeepLast(1));
//   qos.transient_local().reliable();

//   this->goal_checker_selector_pub_ =
//     getNode()->create_publisher<std_msgs::msg::String>(goal_checker_selector_topic_, qos);
}

void CpVisitedPointsCounter::countVisitedPoint()
{
//   setGoalCheckerId(default_goal_checker_name_);  // default id in navigation2 stack
    visited_points_counter_++;
    // RCLCPP_INFO_STREAM(
    //     getLogger(), "[CpVisitedPointsCounter] Visited points: " << visited_points_counter_);
}

int CpVisitedPointsCounter::getVisitedPoints()
{
    

    RCLCPP_FATAL(getLogger(), "[CpVisitedPointsCounter] VISITED POINTS: %d", visited_points_counter_);

    // if(getNode()->get_parameter("max_visited_points", maxVisit)){
      if( visited_points_counter_ >= max_visit_){
        this->postEvent<EvExplorationFinished>();
        RCLCPP_FATAL(getLogger(), "[CpVisitedPointsCounter] EVENT RAISED: EXPLORATION FINISHED");
      }
    // }
    return visited_points_counter_;
}

int CpVisitedPointsCounter::getMaximumVisitedPoints()
{
    return max_visit_;
}

}  // namespace cl_rrt_explore_assigner

// } // namespace sm_isaac_exploration

// EvExplorationFinished : sc::event<EvExplorationFinished>{};
// void CpVisitedPointsCounter::setGoalCheckerId(std::string goalcheckerid)
// {
//   RCLCPP_INFO_STREAM(
//     getLogger(), "[CpGoalCheckerSwitcher] Setting goal checker: " << goalcheckerid);

//   // controller_server_node_->wait_for_service();
//   // std::vector<rclcpp::Parameter> params{ rclcpp::Parameter("current_goal_checker", goalcheckerid) };
//   // auto futureResults = controller_server_node_->set_parameters(params);

//   std_msgs::msg::String msg;
//   msg.data = goalcheckerid;
//   this->goal_checker_selector_pub_->publish(msg);

//   // int i = 0;
//   // for (auto& res : futureResults.get())
//   // {
//   //   RCLCPP_INFO_STREAM(getLogger(), "[CpGoalCheckerSwitcher] parameter result: "
//   //                                                   << params[i].get_name() << "=" << params[i].as_string()
//   //                                                   << ". Result: " << res.successful);
//   //   i++;

//   //   if (!res.successful)
//   //     RCLCPP_ERROR_STREAM(this->getLogger(), "[CpGoalCheckerSwitcher] goal checker could not properly
//   //     switch "
//   //                                                        "the goal checker of the controller_server");
//   // }
// }

// }  // namespace cl_nav2z
