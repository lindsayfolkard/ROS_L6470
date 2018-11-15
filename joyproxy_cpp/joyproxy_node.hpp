#pragma once

/// JoyProxy c++ Node
/// @author Lindsay Folkard
/// @date
/// @description
///
///

// Includes
#include <chrono>
#include <boost/optional.hpp>

// Ros 2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "rcl/rcl.h"
#include "sensor_msgs/msg/joy.hpp"

// Interface libs
#include "l6470_msgs/msg/multi_status.hpp"
#include "l6470_msgs/msg/multi_pose.hpp"
#include "l6470_msgs/msg/manual_speed.hpp"
#include "l6470_srvs/srv/go_to_position.hpp"
#include "l6470_srvs/srv/stop.hpp"
#include "std_msgs/msg/string.hpp"

// STL Libraries
#include <memory>
#include <mutex>

using namespace std::chrono_literals;

namespace joyproxy
{

struct SpeedData
{
    l6470_msgs::msg::ManualSpeed          data;
    std::chrono::steady_clock::time_point time;
};

class JoyProxy : public rclcpp::Node
{

public:

  JoyProxy();
  ~JoyProxy();

protected:

  void joyCallback(const sensor_msgs::msg::Joy::UniquePtr joyMsgPtr);

private:

  // Publishers
  rclcpp::Publisher<l6470_msgs::msg::ManualSpeed>::SharedPtr speedPublisher_;

  // Simple string publisher (to test comms theory)
  std::shared_ptr<std_msgs::msg::String> msg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_ = 1;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySub_;

  boost::optional<SpeedData> speedData_;
  const int timeoutMs_ = 350;

};

}




