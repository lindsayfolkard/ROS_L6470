#include <iostream> // TODO - lets remove and just use ros logging output (much richer)
#include <stdexcept>
#include <string>
#include <assert.h>
#include <chrono>
#include <functional>
#include <fstream>
#include <streambuf>
#include "joyproxy_node.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace joyproxy
{

JoyProxy::JoyProxy():
    Node("JoyProxy")
{
    std::cout << "Create JoyProxy c++ Node" << std::endl;

    // Create a publisher of l6470_msgs::msg::Pose msgs
    speedPublisher_  = create_publisher<l6470_msgs::msg::ManualSpeed>("manual_speed");

    // Handle JoyStick Subscription
    auto f = std::bind(&JoyProxy::joyCallback,this,std::placeholders::_1);
    joySub_ = this->create_subscription<sensor_msgs::msg::Joy> ("joy",f);

    std::cout << "Created JoyProxy c++ node" << std::endl;

    //    // Hack ! - add normal string publisher code
    //    msg_ = std::make_shared<std_msgs::msg::String>();
    //        auto publish_message =
    //          [this]() -> void
    //          {
    //            msg_->data = "Hello World: " + std::to_string(count_++);
    //            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_->data.c_str());

    //            // Put the message into a queue to be processed by the middleware.
    //            // This call is non-blocking.
    //            pub_->publish(msg_);
    //          };

    //        // Create a publisher with a custom Quality of Service profile.
    //        rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    //        custom_qos_profile.depth = 7;
    //        const std::string topicName = "chatter";
    //        pub_ = this->create_publisher<std_msgs::msg::String>(topicName, custom_qos_profile);

    //        // Use a timer to schedule periodic message publishing.
    //    timer_ = this->create_wall_timer(1s, publish_message);

    // END Hack!
}

JoyProxy::~JoyProxy()
{
    std::cout << "Destroy JoyProxy c++ Node" << std::endl;
}

void
JoyProxy::joyCallback(const sensor_msgs::msg::Joy::UniquePtr joyMsgPtr)
{  
    //    if (!joyMsgPtr.get())
    //        throw std::invalid_argument("Joy Msg is nullptr");

    std::stringstream ss;
    ss << "Got Joy Msg. Axes = [";
    for (auto &axes : joyMsgPtr->axes)
    {
        ss << axes << ",";
    }
    ss << "]" << std::endl;

    std::cout << ss.str();

    // There is a potential for speed spamming. Let's only update the speed if it has been long enough
    const std::chrono::milliseconds timeThreshold(100);

    if (!speedData_)
        RCLCPP_INFO(this->get_logger(), "Manual Control Enabled");

    if (speedData_ && ((std::chrono::steady_clock::now() - speedData_->time) < timeThreshold))
    {
        RCLCPP_INFO(this->get_logger(),"Debug - not enough time has passed to publish a new manual speed command");
        return;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Publish!");
    }

    SpeedData speedData;

    speedData.time = std::chrono::steady_clock::now();

    // Create an appropriate manual speed command
    //
    const int expectedMinimumSize = 3;
    if (joyMsgPtr->axes.size() < expectedMinimumSize)
       throw std::runtime_error("Expect joy axes to have length > " + std::to_string(expectedMinimumSize) + " but got length " + std::to_string(joyMsgPtr->axes.size()));

    // Scaling
    // Well - the joystick values are normalised in the range -1.0:1.0
    const int maxStepsPerSecond = 800;

    // Generate the speed request command according to the above mapping
    speedData.data.speed.push_back(maxStepsPerSecond*joyMsgPtr->axes[1]);
    speedData.data.speed.push_back(maxStepsPerSecond*joyMsgPtr->axes[4]);
    speedData.data.speed.push_back(maxStepsPerSecond*joyMsgPtr->axes[2]);

    // Publish
    speedPublisher_->publish(speedData.data);
    speedData_ = speedData;
}

} // joyproxy namespace

#include "class_loader/register_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.

CLASS_LOADER_REGISTER_CLASS(joyproxy::JoyProxy, rclcpp::Node)
