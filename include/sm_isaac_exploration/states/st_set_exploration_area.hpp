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

#include <smacc2/client_behaviors/cb_wait_action_server.hpp>
#include <smacc2/smacc.hpp>

namespace sm_isaac_exploration {
using namespace smacc2::default_events;
using smacc2::client_behaviors::CbSleepFor;
using namespace std::chrono_literals;
using cl_rrt_explore_assigner::CbStartExploration;
using smacc2::client_behaviors::CbRosLaunch2;
using smacc2::client_behaviors::CbWaitActionServer;

// STATE DECLARATION
struct StSetExplorationArea
    : smacc2::SmaccState<StSetExplorationArea, MsIsaacExplorationRunMode> {
  using SmaccState::SmaccState;

  struct STARTING_EXPLORATION : SUCCESS {};
  // TRANSITION TABLE
  typedef mpl::list<Transition<EvCbSuccess<CbWaitActionServer, OrAssigner>,
                               StExploreNextPoint, STARTING_EXPLORATION>,
                    Transition<EvCbSuccess<CbSleepFor, OrAssigner>,
                               StExploreNextPoint, STARTING_EXPLORATION>>
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {
    configure_orthogonal<OrAssigner, CbStartExploration>();
    configure_orthogonal<OrAssigner, CbWaitActionServer>(90s);
    configure_orthogonal<OrAssigner, CbSleepFor>(60s);
    configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>(0.0, 0, 0);
  }
};
} // namespace sm_isaac_exploration
