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
using namespace smacc2::default_events;
using smacc2::client_behaviors::CbSleepFor;
using smacc2::client_behaviors::CbRosStop2;
using cl_lifecyclenode::CbDeactivate;
using namespace std::chrono_literals;

// STATE DECLARATION
struct StFinalMapSaving
    : smacc2::SmaccState<StFinalMapSaving, MsIsaacExplorationRunMode> {
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
        // Transition<EvCbSuccess<CbSaveSlamMap, OrNavigation>, SS2::SsRadialPattern2, SUCCESS>,
        // Transition<EvCbSuccess<CbSleepFor, OrNavigation>, SS2::SsRadialPattern2, SUCCESS>,
        Transition<EvCbSuccess<CbSleepFor, OrLocalization>, SS2::SsRadialPattern2, SUCCESS>
                    >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {
    configure_orthogonal<OrNavigation, CbActiveStop>();
    configure_orthogonal<OrNavigation, CbSaveSlamMap>();
    configure_orthogonal<OrNavigation, CbSleepFor>(10s);
    // configure_orthogonal<OrSlam, CbRosStop2>();
    configure_orthogonal<OrSlam, CbPauseSlam>();
    configure_orthogonal<OrNavigation, CbSleepFor>(10s);
    // configure_orthogonal<OrLocalization, CbRosLaunch2>(
    //     "sm_isaac_exploration",
    //     "isaac_ros_occupancy_grid_localizer_nav2.launch.py",
    //     smacc2::client_behaviors::RosLaunchMode::LAUNCH_DETTACHED);
    configure_orthogonal<OrLocalization, CbRosLaunch2>(
        "sm_isaac_exploration",
        // "isaac_ros_occupancy_grid_localizer",
        "isaac_ros_occupancy_grid_localizer_nav2.launch.py",
        smacc2::client_behaviors::RosLaunchMode::LAUNCH_DETTACHED);
    configure_orthogonal<OrLocalization, CbSleepFor>(15s);
  }

  void runtimeConfigure() {}
};
} // namespace sm_isaac_exploration
