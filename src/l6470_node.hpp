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
//#include "l6470_driver/types.h"
//#include "l6470_driver/commands.h"

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

  rclcpp::Publisher<ros_l6470_msgs::msg::Pose>::SharedPtr posePublisher_;
  rclcpp::timer::TimerBase::SharedPtr timer_;

  // L6470 Multi Driver
  std::unique_ptr<MultiDriver> driver_;

};

}  // namespace l6470




