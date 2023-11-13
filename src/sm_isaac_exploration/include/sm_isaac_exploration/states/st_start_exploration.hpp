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

namespace sm_isaac_exploration {
using namespace smacc2::default_events;
using smacc2::client_behaviors::CbSleepFor;
using namespace std::chrono_literals;
using cl_rrt_explore_assigner::CbStartExploration;
// using smacc2::client_behaviors::CbRosLaunch2;

// STATE DECLARATION
struct StLaunchExploration
    : smacc2::SmaccState<StLaunchExploration, MsIsaacExplorationRunMode> {
  using SmaccState::SmaccState;

  struct STARTING_EXPLORATION : SUCCESS {};
  struct SrTimeTopic;
  // TRANSITION TABLE
  typedef mpl::list<
      // Transition<EvCbSuccess<CbSleepFor, OrAssigner>, StSetExplorationArea,
      // STARTING_EXPLORATION>
      // Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>,
      //            StSetExplorationArea, STARTING_EXPLORATION>
      Transition<EvAllGo<SrAllEventsGo, SrTimeTopic>, StSetExplorationArea,
                 STARTING_EXPLORATION>
                 
      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {
    // configure_orthogonal<OrPerception, CbRosLaunch2>("sm_isaac_exploration","isaac_ros_apriltag_isaac_sim_pipeline.launch.py",smacc2::client_behaviors::RosLaunchMode::LAUNCH_CLIENT_BEHAVIOR_LIFETIME);
    // configure_orthogonal<OrNavigation, CbRosLaunch2>("sm_isaac_exploration", "slam_launch.py", smacc2::client_behaviors::RosLaunchMode::LAUNCH_DETTACHED);
    configure_orthogonal<OrAssigner, CbRosLaunch2>("rrt_exploration", "simple.launch.py", smacc2::client_behaviors::RosLaunchMode::LAUNCH_DETTACHED);
    
    configure_orthogonal<OrAssigner, CbSleepFor>(10s);
    configure_orthogonal<OrAssigner, CbWaitTopic>("/filtered_points");

    
    // configure_orthogonal<OrNavigation, CbPureSpinning>(M_PI*2);
    configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>(0.5, 0, 0);

    auto srTimeTopic = static_createStateReactor<
        SrAllEventsGo,
        smacc2::state_reactors::EvAllGo<SrAllEventsGo, SrTimeTopic>,
        mpl::list<
            EvCbSuccess<CbSleepFor, OrAssigner>,
            EvCbSuccess<CbWaitTopic, OrAssigner>,
            EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>
            >>();
  }
};
} // namespace sm_isaac_exploration
