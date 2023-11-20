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
#include <smacc2/client_behaviors/cb_ros_launch_2.hpp>
#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.hpp>
#include <sm_isaac_exploration/clients/cl_april_tag_detector/client_behaviors/cb_detect_apriltag.hpp>


namespace sm_isaac_exploration 
{
using namespace smacc2::default_events;
using namespace std::chrono_literals;

using cl_apriltag_detector::CbDetectAprilTag;
using cl_keyboard::CbDefaultKeyboardBehavior;
using cl_nav2z::CbResumeSlam;
using cl_rrt_explore_assigner::CbRrtExploreNextPoint;
using smacc2::client_behaviors::CbSleepFor;
using namespace cl_rrt_explore_assigner;
// using namespace cl_rrt_explore_assigner::CpVisitedPointsCounter;

// struct EvExplorationFinished : sc::event<EvExplorationFinished> {};

// STATE DECLARATION
struct StExploreNextPoint
    : smacc2::SmaccState<StExploreNextPoint, MsIsaacExplorationRunMode> {
  using SmaccState::SmaccState;
  // DECLARE CUSTOM OBJECT AND TRANSITION TAGS
  // struct SrAcquireSensors;
  // struct LAST_POINT_EXPLORED : SUCCESS {};
  struct SrLastPointExplored;
  // TRANSITION TABLE
  typedef mpl::list<
      // Transition<EvCbSuccess<CbRrtExploreNextPoint, OrNavigation>, StExplorationPointSpinning, SUCCESS>,
      
      Transition<EvCbSuccess<CbRrtExploreNextPoint, OrNavigation>, StExploreCheckPoint, SUCCESS>,
      Transition<EvCbFailure<CbRrtExploreNextPoint, OrNavigation>, StExploreNextPoint, ABORT>,

      Transition<cl_keyboard::EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>,StFinalReturnBackToOrigin, SUCCESS>,
      Transition<cl_keyboard::EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>,StExploreNextPoint, SUCCESS>,
      Transition<cl_keyboard::EvKeyPressE<CbDefaultKeyboardBehavior, OrKeyboard>,StFinalReturnBackToOrigin, SUCCESS>
      >
      reactions;

  static int count_visited_states;
  const int EXPLORE_POINTS_COUNT = 50;
  CpVisitedPointsCounter *visitedCount;

  // STATE FUNCTIONS
  static void staticConfigure() {
    // visitedCount = CpVisitedPointsCounter::CpVisitedPointsCounter();
    configure_orthogonal<OrNavigation, CbRrtExploreNextPoint>();
    configure_orthogonal<OrPerception, CbDetectAprilTag>();
    configure_orthogonal<OrPerception, CbResumeSlam>();
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
    // configure_orthogonal<OrAssigner, CpVisitedPointsCounter>();

    // auto srAllPointsExplored = static_createStateReactor<
    //     SrAllEventsGo,
    //     smacc2::state_reactors::EvAllGo<SrAllEventsGo, SrLastPointExplored>,
    //     mpl::list<
    //         EvCbSuccess<CbRrtExploreNextPoint, OrNavigation>,
    //         EvExplorationFinished
    //         >>();
  }

  void runtimeConfigure() {}

  void onEntry() {
    // cl_rrt_explore_assigner::ClRrtExploreAssigner* assigner;
    // this->requiresClient(assigner);
    // // if number of explorations > 50 post event
    // // if (count_visited_states > EXPLORE_POINTS_COUNT) {
    // //   this->postEvent<EvExplorationFinished>();
    // // }
    // // int maxVisit;
    // visitedCount = assigner->getComponent<CpVisitedPointsCounter>();
    // visitedCount->getVisitedPoints();
    // // if(getNode()->get_parameter("max_visited_points", maxVisit)){
    // //   if(visitedCount->getVisitedPoints() >= maxVisit){
    // //     visitedCount->postEvent<EvExplorationFinished>();
    // //   }
    // // }
    // // if(visitedCount->getVisitedPoints() >= getNode()->get_parameter("max_visited_points")){}
  }

  void onExit(SUCCESS) {
    // visitedCount->countVisitedPoint();
    // // count_visited_states++;
    // // int maxVisit;
    // // getNode()->get_parameter("max_visited_points", maxVisit);
    // auto maxint = 0;
    // getNode()->get_parameter("max_visited_points", maxint);

    // RCLCPP_INFO_STREAM(getLogger(), "[StExploreNextPoint] MAX FROM STATE: " << maxint);

    //  RCLCPP_FATAL(getLogger(), "[StExploreNextPoint] Visited states: %d/%d",
    //              visitedCount->getVisitedPoints(), visitedCount->getMaximumVisitedPoints());

    // // cl_apriltag_detector::ClAprilTagDetector *cbtag;
    // // requiresClient(cbtag);

    // // RCLCPP_INFO(getLogger(), "[StExploreNextPoint] detected tags: %d",
    // //             cbtag->detectedAprilTagsMapPose_.size());
  }
};
} // namespace sm_isaac_exploration
