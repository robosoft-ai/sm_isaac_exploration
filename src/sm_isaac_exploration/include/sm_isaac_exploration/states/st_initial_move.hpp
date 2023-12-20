#pragma once

#include <smacc2/smacc.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <smacc2/client_base_components/cp_topic_publisher.hpp>

namespace sm_isaac_exploration {
using namespace smacc2::default_events;
using smacc2::client_behaviors::CbSleepFor;
using namespace std::chrono_literals;
using namespace cl_nav2z;


// STATE DECLARATION
struct StInitialMove
    : smacc2::SmaccState<StInitialMove, MsIsaacExplorationRunMode> {
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbSleepFor, OrNavigation>, StStartExploration, SUCCESS>
  > reactions;

//   CpTopicPublisher<geometry_msgs::msg::Twist> *pub;

  // STATE FUNCTIONS
  static void staticConfigure() {
    configure_orthogonal<OrNavigation, CbSleepFor>(10s);
  }

  void runtimeConfigure() {}

  void onEntry() {
    cl_nav2z::ClNav2Z* clNav;
    this->requiresClient(clNav);
    auto pub = clNav->getComponent<smacc2::components::CpTopicPublisher<geometry_msgs::msg::Twist>>();
    auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
    twist_msg->linear.x = 0.2; 
    twist_msg->angular.z = 1.0; 
    pub->publish(*twist_msg);
  }
};
} 