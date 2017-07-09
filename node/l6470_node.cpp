
#include "l6470_node.hpp"
#include "l6470_driver/types.h"
#include "l6470_driver/commands.h"
#include <iostream> // TODO - lets remove and just use ros logging output (much richer)
#include <stdexcept>
#include <string>
#include <assert.h>
#include <chrono>
#include <functional>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace l6470
{

L6470Node::L6470Node():
    Node("l6470node"),
    count_(0)
{
    // TODO - instantiate the driver ?? (or leave this for the init stage --> yes , this should be in the init stage)
    std::cout << "Creating L6470 Node!"<<std::endl;

    // Create a publisher of l6470_msgs::msg::Pose msgs
    posePublisher_ = create_publisher<l6470_msgs::msg::MultiPose>("multipose");

    // Use a timer to schedule periodic message publishing.
    std::chrono::milliseconds period(500);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&L6470Node::on_timer, this));

    // Handle Motor Speed Message Subscription
    auto f = std::bind(&L6470Node::manualSpeedCallback,this,_1);
    speedSub_ = this->create_subscription<l6470_msgs::msg::ManualSpeed> ("manual_speed",f);

    // Handle goToPosition service
    auto c = std::bind(&L6470Node::goToPositionCallback,this,_1,_2);
    goToPositionSrv_ = this->create_service<l6470_srvs::srv::GoToPosition> ("go_to_position",c);

    // Handle stop service
    auto d = std::bind(&L6470Node::stopCallback,this,_1,_2);
    stopSrv_ = this->create_service<l6470_srvs::srv::Stop> ("stop",d);
    std::cout << "Created L6470 Node!" << std::endl;
}

L6470Node::~L6470Node()
{
    std::cout << "Destroy L6470 Node" << std::endl;
}

void
L6470Node::on_timer()
{
    // Send Speed Commands (if needed)

    // Let's just make a fake message for now ?
    auto msg = std::make_shared<l6470_msgs::msg::MultiPose>();

    auto motorP1 = l6470_msgs::msg::Pose();
    motorP1.position=++count_;
    motorP1.speed=count_*2;
    msg->motor_states.push_back(motorP1);

    //msg->motor_states[0].position=++count_;
    //msg->motor_states[0].speed=count_*2;
    std::cout << "Publishing: message " << count_ << std::endl;
    std::flush(std::cout);

    // Put the message into a queue to be processed by the middleware.
    // This call is non-blocking.
    posePublisher_->publish(msg);

    // Handle Status Message publishing (at a lower frequency : or I could integrate the status in the state/pose struct ?) ??
    // #TODO

}

void
L6470Node::manualSpeedCallback(const l6470_msgs::msg::ManualSpeed::UniquePtr manualSpeed)
{
    std::cout << "DEBUG - set manual speed command ... TODO" << std::endl;
    //std::cout << "DEBUG : set manual speed" << manualSpeed.speed[0] << "steps per second" << std::endl;
    // TODO
}

void
L6470Node::goToPositionCallback(const std::shared_ptr <l6470_srvs::srv::GoToPosition::Request>  request,
                                      std::shared_ptr <l6470_srvs::srv::GoToPosition::Response> response
                                )
{
    std::cout << "GOToPosition callback .. TODO" << std::endl;
   // TODO
}

void
L6470Node::stopCallback(const std::shared_ptr <l6470_srvs::srv::Stop::Request>  request,
                              std::shared_ptr <l6470_srvs::srv::Stop::Response> response)
{
    std::cout << "STOP callback .. TODO" << std::endl;
}

} // l6470 namespace

#include "class_loader/class_loader_register_macro.h"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.

CLASS_LOADER_REGISTER_CLASS(l6470::L6470Node, rclcpp::Node)
