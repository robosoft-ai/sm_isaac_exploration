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

#include <smacc2/smacc.hpp>

namespace sm_isaac_exploration {

using namespace std::chrono_literals;
using namespace smacc2::default_events;
using cl_nav2z::CbNavigateGlobalPosition;
using smacc2::client_behaviors::CbSleepFor;

// STATE DECLARATION
struct StNavigateToAprilTagWorkArea
    : smacc2::SmaccState<StNavigateToAprilTagWorkArea, MsIsaacExplorationRunMode> 
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
        Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, SS4::SsFPattern1, SUCCESS>,
        Transition<EvCbFailure<CbNavigateGlobalPosition, OrNavigation>, StNavigateToAprilTagWorkArea, ABORT>
      >
      reactions;

  cl_apriltag_detector::ClAprilTagDetector *tagDetectorClient;

  // STATE FUNCTIONS
  static void staticConfigure() {
    configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>();
  }

  void runtimeConfigure() {
    this->requiresClient(tagDetectorClient);

    std::string tagToVisit = *tagDetectorClient->selectedVisitTagId_;
    auto tagPose = tagDetectorClient->detectedAprilTagsMapPose_[tagToVisit];

    auto cbnav = this->getOrthogonal<OrNavigation>()
                     ->getClientBehavior<CbNavigateGlobalPosition>();
    cbnav->setGoal(tagPose.pose);
  }

  void onExit(SUCCESS) {
    tagDetectorClient->visitedWorkingAreas_.push_back(
        *tagDetectorClient->selectedVisitTagId_);
    tagDetectorClient->selectedVisitTagId_ = std::nullopt;
  }
};
} // namespace sm_isaac_exploration
