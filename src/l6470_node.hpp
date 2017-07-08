#pragma once

/// L6470 Node
/// @author Lindsay Folkard
/// @date
/// @description
///
///

// Includes

// L6470 Driver
#include "l6470_driver/multidriver.h"

// Ros 2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "rcl/rcl.h"

// Msg's and interface libs
#include "ros_l6470_msgs/msg/pose.hpp"

// STL Libraries
#include <memory>

namespace l6470
{

class L6470Node : public rclcpp::Node
{

public:

  L6470Node();

protected:

  void on_timer();

private:

  // test variables
  int count_;

  // Publishers
  rclcpp::Publisher<ros_l6470_msgs::msg::MultiPose>::SharedPtr   posePublisher_;
  rcpcpp::Publisher<ros_l6470_msgs::msg::MultiStatus>::SharedPtr statusPublisher_;
  //rcpcpp::Publisher<ros_l6470_msgs::msg::MultiStatus>

  // Subscriptions
  rclcpp::subscription::Subscription<ros_l6470_msgs::msg::ManualSpeed>::SharedPtr speedSub_;

  // Services
  //rclcpp::service::Service<example_interfaces::srv::AddTwoInts>::SharedPtr srv_;

  // Wall Timers
  rclcpp::timer::TimerBase::SharedPtr timer_; // I prefer if there is really only one task that talks to the controller
  //rclcpp::timer::TimerBase::SharedPtr statusTimer_;

  // L6470 Multi Driver
  std::unique_ptr<MultiDriver> driver_;

};

}  // namespace l6470




