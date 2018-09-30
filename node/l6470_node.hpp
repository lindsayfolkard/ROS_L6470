#pragma once

/// L6470 Node
/// @author Lindsay Folkard
/// @date
/// @description
///
///

// Includes
#include <chrono>
#include <boost/optional.hpp>

// Driver
#include "l6470_driver/abstractdriver.h"

// Ros 2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "rcl/rcl.h"

// Interface libs
#include "l6470_msgs/msg/multi_status.hpp"
#include "l6470_msgs/msg/multi_pose.hpp"
#include "l6470_msgs/msg/manual_speed.hpp"
#include "l6470_srvs/srv/go_to_position.hpp"
#include "l6470_srvs/srv/stop.hpp"
//#include "l6470_srvs/srv/gospeed.hpp"

// STL Libraries
#include <memory>
#include <mutex>

namespace l6470
{

class L6470Node : public rclcpp::Node
{

public:

  L6470Node();
  ~L6470Node();

protected:

  void poseTimerCallback();

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
  rclcpp::Subscription<l6470_msgs::msg::ManualSpeed>::SharedPtr speedSub_;

  // Services
  rclcpp::Service<l6470_srvs::srv::GoToPosition>::SharedPtr goToPositionSrv_;
  rclcpp::Service<l6470_srvs::srv::Stop>::SharedPtr         stopSrv_;

  // Wall Timers
  rclcpp::TimerBase::SharedPtr poseTimer_;
  rclcpp::TimerBase::SharedPtr statusTimer_;

  // Stepper Motor Driver
  std::mutex driverMutex_;
  std::unique_ptr<AbstractDriver> driver_;

  // Time outs for manual control
  boost::optional<std::chrono::steady_clock::time_point> lastManualSpeedUpdate_;

};

}  // namespace l6470




