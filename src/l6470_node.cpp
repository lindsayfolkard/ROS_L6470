
#include "l6470_node.hpp"
#include <iostream> // TODO - lets remove and just use ros logging output (much richer)
//#include <stdexcept>
//#include <string>
//#include <assert.h>
#include <chrono>

using namespace std::chrono_literals;

namespace l6470
{

L6470Node::L6470Node():
    Node("l6470node"),
    count_(0)
{
    // TODO - instantiate the driver ?? (or leave this for the init stage --> yes , this should be in the init stage)

    // Create a publisher of ros_l6470_msgs::msg::Pose msgs
    posePublisher_ = create_publisher<ros_l6470_msgs::msg::Pose>("pose");

    // Use a timer to schedule periodic message publishing.
    timer_ = create_wall_timer(1s, std::bind(&L6470Node::on_timer, this));
}

void
L6470Node::on_timer()
{
    auto msg = std::make_shared<ros_l6470_msgs::msg::Pose>();
    msg->position=++count_;
    msg->speed=count_*2;
    std::cout << "Publishing:" << msg << std::endl;
    std::flush(std::cout);

    // Put the message into a queue to be processed by the middleware.
    // This call is non-blocking.
    posePublisher_->publish(msg);
}

} // l6470 namespace

#include "class_loader/class_loader_register_macro.h"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.

CLASS_LOADER_REGISTER_CLASS(l6470::L6470Node, rclcpp::Node)
