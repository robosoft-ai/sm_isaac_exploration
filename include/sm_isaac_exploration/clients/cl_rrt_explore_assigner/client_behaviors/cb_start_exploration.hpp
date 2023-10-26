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
#include <rrt_exploration/action/explore_next_point.hpp>
#include <sm_isaac_exploration/clients/cl_rrt_explore_assigner/cl_rrt_explore_assigner.hpp>
#include <smacc2/client_base_components/cp_topic_publisher.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_rrt_explore_assigner {
using namespace std::chrono_literals;
class CbStartExploration : public smacc2::SmaccAsyncClientBehavior {
private:
  ClRrtExploreAssigner *exploreNextPointClient_;

public:
  CbStartExploration() {}

  virtual ~CbStartExploration() {}

  virtual void onEntry() override {
    this->requiresClient(this->exploreNextPointClient_);
    auto pointspublisher =
        this->exploreNextPointClient_
            ->getComponent<smacc2::components::CpTopicPublisher<
                geometry_msgs::msg::PointStamped>>();

    std::vector<geometry_msgs::msg::PointStamped> points;

    double scale = 6.0;

    while (!this->isShutdownRequested()) {
      rclcpp::sleep_for(5s);

      points.push_back(createPoint(-30.2908992767334*scale, scale*44.03512191772461, scale*0.006378173828125));
      points.push_back(createPoint(29.334697723388672*scale, 43.96137237548828*scale, 0.002471923828125*scale));
      points.push_back(createPoint(28.936553955078125* scale, -30.039058685302734*scale,-0.001434326171875* scale));
      points.push_back(createPoint(-29.617624282836914* scale, -29.65144920349121*scale,-0.001434326171875*scale));
      points.push_back(createPoint(-0.2395467907190323*scale, 0.038018785417079926*scale,0.00247192382*scale));

      // Publicar todos los puntos en la lista
      for (const auto &point : points) {
        pointspublisher->publish(point);
      }
      this->postSuccessEvent();
    }
  }

  geometry_msgs::msg::PointStamped createPoint(double x, double y, double z) {
    geometry_msgs::msg::PointStamped point;
    point.header.frame_id = "map";
    point.header.stamp = rclcpp::Clock().now(); // Utilizar la hora actual
    point.point.x = x;
    point.point.y = y;
    point.point.z = z;
    return point;
  }
};
} // namespace cl_rrt_explore_assigner
