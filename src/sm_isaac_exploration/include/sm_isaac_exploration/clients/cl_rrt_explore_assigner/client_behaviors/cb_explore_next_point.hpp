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

#include <rrt_exploration/action/explore_next_point.hpp>
#include <sm_isaac_exploration/clients/cl_rrt_explore_assigner/cl_rrt_explore_assigner.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_rrt_explore_assigner {
class CbRrtExploreNextPoint : public smacc2::SmaccAsyncClientBehavior {
private:
  ClRrtExploreAssigner *exploreNextPointClient_;
  ClRrtExploreAssigner::SmaccExploreNextResultSignal::SharedPtr
      exploreCallback_;

public:
  CbRrtExploreNextPoint() {}

  virtual void onEntry() override {
    this->requiresClient(this->exploreNextPointClient_);

    this->exploreCallback_ =
        std::make_shared<ClRrtExploreAssigner::SmaccExploreNextResultSignal>();

    this->getStateMachine()->createSignalConnection(
        *exploreCallback_, &CbRrtExploreNextPoint::onActionServerResult, this);

    ClRrtExploreAssigner::Goal goal;
    // try {
        auto fut = this->exploreNextPointClient_->sendGoal(goal, this->exploreCallback_);
    //     auto ret = fut.get();
    // }catch(std::runtime_error &e){
    //       RCLCPP_INFO_STREAM(getLogger(), "Action goal failure");
    // }
  }

  void onActionServerResult(const ClRrtExploreAssigner::WrappedResult &r) {
    RCLCPP_INFO_STREAM(getLogger(), "Action server response received");
    if (r.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO_STREAM(getLogger(), "SUCCEEDED");
      this->postSuccessEvent();
    } else if (r.code == rclcpp_action::ResultCode::ABORTED) {
      RCLCPP_INFO_STREAM(getLogger(), "ABORTED");
      this->postFailureEvent();
    } else if (r.code == rclcpp_action::ResultCode::CANCELED) {
      RCLCPP_INFO_STREAM(getLogger(), "CANCELED");
      this->postFailureEvent();
    } else {
      RCLCPP_INFO_STREAM(getLogger(), "FAILED");
      this->postFailureEvent();
    }
  }
};
} // namespace cl_rrt_explore_assigner
