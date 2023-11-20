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

#include <geometry_msgs/msg/point_stamped.hpp>
#include <sm_isaac_exploration/clients/cl_rrt_explore_assigner/cl_rrt_explore_assigner.hpp>
#include <smacc2/client_base_components/cp_topic_publisher.hpp>
#include <smacc2/client_bases/smacc_ros_launch_client_2.hpp>
#include <sm_isaac_exploration/clients/cl_rrt_explore_assigner/components/cp_visited_points_counter.hpp>
#include <smacc2/smacc.hpp>

namespace sm_isaac_exploration {
using namespace std::chrono_literals;

class OrAssigner : public smacc2::Orthogonal<OrAssigner> {
public:
  void onInitialize() override {
    auto assignerclient =
        this->createClient<cl_rrt_explore_assigner::ClRrtExploreAssigner>(
            "/explore_next_point");
    assignerclient->createComponent<
        smacc2::components::CpTopicPublisher<geometry_msgs::msg::PointStamped>>(
        "/clicked_point");

    assignerclient->createComponent<cl_rrt_explore_assigner::CpVisitedPointsCounter>();

    this->createClient<smacc2::client_bases::ClRosLaunch2>();
  }
};
} // namespace sm_isaac_exploration
