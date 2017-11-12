#pragma once

/// L6470 Node
/// @author Lindsay Folkard
/// @date
/// @description
///
///

// Includes

// L6470 Driver
//#include "l6470_driver/multidriver.h"
#include "l6470_driver/basedriver.h"

// Ros 2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "rcl/rcl.h"

// Interface libs
#include "l6470_msgs/msg/multi_status.hpp"
#include "l6470_msgs/msg/multi_pose.hpp"
#include "l6470_msgs/msg/manual_speed.hpp"
#include "l6470_srvs/srv/go_to_position.hpp"
#include "l6470_srvs/srv/stop.hpp"

// STL Libraries
#include <memory>

namespace l6470
{

class L6470Node : public rclcpp::Node
{

public:

  L6470Node();
  ~L6470Node();

protected:

  void on_timer();

  void manualSpeedCallback(const l6470_msgs::msg::ManualSpeed::UniquePtr manualSpeed);

  void goToPositionCallback(const std::shared_ptr <l6470_srvs::srv::GoToPosition::Request>  request,
                                  std::shared_ptr <l6470_srvs::srv::GoToPosition::Response> response);
  void stopCallback(const std::shared_ptr <l6470_srvs::srv::Stop::Request>  request,
                          std::shared_ptr <l6470_srvs::srv::Stop::Response> response);

private:

  // test variables
  int count_;

  // Publishers
  rclcpp::Publisher<l6470_msgs::msg::MultiPose>::SharedPtr   posePublisher_;
  rclcpp::Publisher<l6470_msgs::msg::MultiStatus>::SharedPtr statusPublisher_;

  // Subscriptions
  rclcpp::subscription::Subscription<l6470_msgs::msg::ManualSpeed>::SharedPtr speedSub_;

  // Services
  rclcpp::service::Service<l6470_srvs::srv::GoToPosition>::SharedPtr goToPositionSrv_;
  rclcpp::service::Service<l6470_srvs::srv::Stop>::SharedPtr         stopSrv_;

  // Wall Timers
  rclcpp::timer::TimerBase::SharedPtr timer_; // I prefer if there is really only one task that talks to the controller

  // L6470 Multi Driver
  //std::unique_ptr<MultiDriver> driver_;

};

}  // namespace l6470




