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

#include <smacc2/client_behaviors/cb_ros_launch_2.hpp>
#include <smacc2/smacc.hpp>
#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.hpp>
#include <sm_isaac_exploration/clients/cl_april_tag_detector/client_behaviors/cb_detect_apriltag.hpp>

namespace sm_isaac_exploration {
using namespace smacc2::default_events;
using smacc2::client_behaviors::CbSleepFor;
using namespace std::chrono_literals;
using cl_keyboard::CbDefaultKeyboardBehavior;
using cl_apriltag_detector::CbDetectAprilTag;

// STATE DECLARATION
struct StExplorationPointSpinning
    : smacc2::SmaccState<StExplorationPointSpinning,
                         MsIsaacExplorationRunMode> {
  using SmaccState::SmaccState;
  // TRANSITION TABLE
  typedef mpl::list<Transition<EvCbSuccess<CbPureSpinning, OrNavigation>, StExploreNextPoint, SUCCESS>,
                    Transition<EvCbFailure<CbPureSpinning, OrNavigation>, StExploreNextPoint, ABORT>,
                    Transition<cl_keyboard::EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StFinalReturnBackToOrigin, SUCCESS>
                    // Transition<cl_apriltag_detector::EvUnvisitedAprilTagDetected<CbDetectAprilTag,
                    // OrPerception>,
                    //                        StPauseSlam, SUCCESS>
                    >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {
    configure_orthogonal<OrNavigation, CbPureSpinning>(M_PI * 2, 0.8);
    configure_orthogonal<OrPerception, CbDetectAprilTag>();
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure() {}
};
} // namespace sm_isaac_exploration
