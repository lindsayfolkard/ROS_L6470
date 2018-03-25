#include "l6470_node.hpp"
#include "l6470_driver/types.h"
#include "l6470_driver/commands.h"
#include "l6470_driver/driverfactory.h"
#include <iostream> // TODO - lets remove and just use ros logging output (much richer)
#include <stdexcept>
#include <string>
#include <assert.h>
#include <chrono>
#include <functional>
#include <fstream>
#include <streambuf>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace l6470
{

namespace
{
l6470_msgs::msg::Status toRosStatus (const Status &status)
{
    l6470_msgs::msg::Status returnStatus;
    returnStatus.has_thermal_warning = status.hasThermalWarning;
    returnStatus.is_busy = status.isBusy;
    returnStatus.is_high_z = status.isHighZ;
    returnStatus.is_in_thermal_shutdown = status.isInThermalShutdown;
    returnStatus.is_switch_closed = status.isSwitchClosed;
    returnStatus.last_command_invalid = status.commandError;
    returnStatus.motor_spin_fwd = (status.spinDirection == Forward);
    returnStatus.over_current_detected = status.overCurrentDetected;
    returnStatus.performed_last_command = status.commandError;
    returnStatus.stall_detected_phase_a = status.stallDetectedPhaseA;
    returnStatus.stall_detected_phase_b = status.stallDetectedPhaseB;
    returnStatus.motor_state = status.motorStatus;

    return returnStatus;
}
}

L6470Node::L6470Node():
    Node("l6470node"),
    count_(0)
{
    // TODO - perhaps put this in the init stage ???
    // Instantiate the Stepper Driver from the factory
    const std::string overallCfgFile = "~/dspin_stepper_configs/ros_overall_config.txt"; // TODO - figure out where this file can be stored/deployed nicely
    std::cout << "Try to instantiate stepper motor driver with ros_overall_config file from " << overallCfgFile << std::endl;
    OverallCfg overallCfg(overallCfgFile);
    driver_ = factoryMakeDriver(overallCfg);
    assert(driver_ || !"driver is null pointer!" );
    std::cout << "Instantiated " << overallCfg.controllerType_ << " stepper motor driver with " << overallCfg.cfgFiles_.size() << " daisy chained steppers" << std::endl;

    // Create a publisher of l6470_msgs::msg::Pose msgs
    posePublisher_ = create_publisher<l6470_msgs::msg::MultiPose>("multipose");

    // Use a timer to schedule periodic message publishing.
    std::chrono::milliseconds period(500);
    poseTimer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&L6470Node::poseTimerCallback, this));

    // Handle Motor Speed Message Subscription
    auto f = std::bind(&L6470Node::manualSpeedCallback,this,std::placeholders::_1);
    speedSub_ = this->create_subscription<l6470_msgs::msg::ManualSpeed> ("manual_speed",f);

    // Handle goToPosition service
    auto c = std::bind(&L6470Node::goToPositionCallback,this,std::placeholders::_1,std::placeholders::_2);
    goToPositionSrv_ = this->create_service<l6470_srvs::srv::GoToPosition> ("go_to_position",c);

    // Handle stop service
    auto d = std::bind(&L6470Node::stopCallback,this,std::placeholders::_1,std::placeholders::_2);
    stopSrv_ = this->create_service<l6470_srvs::srv::Stop> ("stop",d);
    std::cout << "Created L6470 Node!" << std::endl;
}

L6470Node::~L6470Node()
{
    std::cout << "Destroy L6470 Node" << std::endl;
}

void
L6470Node::poseTimerCallback()
{
    // Send Speed Commands (if needed) ? ? ? --> I guess when asking for a new trajectory

    // Let's just make a fake message for now ?
    //    auto msg = std::make_shared<l6470_msgs::msg::MultiPose>();

    //    auto motorP1 = l6470_msgs::msg::Pose();
    //    motorP1.position=++count_;
    //    motorP1.speed=count_*2;

    // Let's get the actual positions and speeds
    std::vector<int>    positions;
    std::vector<int>    speeds;
    std::vector<Status> states;
    {
        std::lock_guard<std::mutex> lock(driverMutex_);
        positions = driver_->getPos();
        speeds = driver_->getSpeed();
        states = driver_->getStatus();
    }

    if (speeds.size() != positions.size())
    {
        std::cout << "Speeds and positions do not match in size (speeds = " << speeds.size() << " while positions length = " << positions.size() << std::endl;
        throw; // throw something more meaningful
    }

    // Create the pose message to publish
    std::shared_ptr<l6470_msgs::msg::MultiPose> poseMsg = std::make_shared<l6470_msgs::msg::MultiPose>();

    for (int motor=0; motor < positions.size(); ++motor)
    {
        poseMsg->motor_states[motor].position = positions[motor];
        poseMsg->motor_states[motor].speed    = speeds[motor];
    }

    std::cout << "Publishing : message " << count_ << std::endl;
    std::flush(std::cout);

    // Create the status message to publish
    std::shared_ptr<l6470_msgs::msg::MultiStatus> statusMsg = std::make_shared<l6470_msgs::msg::MultiStatus>();

    for (const Status &status : states)
    {
        statusMsg->motor_states.push_back(toRosStatus(status));
    }

    // Put the message into a queue to be processed by the middleware.
    // This call is non-blocking.
    posePublisher_->publish(poseMsg);
    statusPublisher_->publish(statusMsg);
}

void
L6470Node::manualSpeedCallback(const l6470_msgs::msg::ManualSpeed::UniquePtr manualSpeed)
{
    // Hack stuff - to prevent compile warning
    if (!manualSpeed.get())
    {
        std::cout << "DEBUG - wtf manualSpeed is nullptr" <<  std::endl;
    }
    else
    {
        // Create an appropriate run command
        //
        std::map <int,RunCommand> runCommands;
        int motor=0;
        for (const auto &speed : manualSpeed->speed)
        {
            runCommands.insert(std::make_pair(motor,RunCommand((speed > 0 ? Forward : Reverse),speed)));
            ++motor;
        }

        std::lock_guard<std::mutex> lock(driverMutex_);

        // Send the command to the motors
        driver_->run(runCommands);
    }
}

void
L6470Node::goToPositionCallback(const   std::shared_ptr <l6470_srvs::srv::GoToPosition::Request>  request,
                                        std::shared_ptr <l6470_srvs::srv::GoToPosition::Response> response )
{
    if (request.get() == nullptr || response.get() == nullptr)
    {
        throw;
        std::cout << "TODO - throw an exception" << std::endl;
    }

    // Get the desired positions
    std::map <int,GoToDirCommand> goToCommands;
    std::map <int,ProfileCfg>  profiles;

    // Check that the vectors are okay
    // TODO - change the command to be a vector or GoToPositiong to avoid this bullshit
    if (!(request->motor_indice.size == request->motor_position.size()
          == request->drive_direction.size()
          == request->acceleration.size()
          == request->deceleration.size()
          == request->speed.size()))
    {
        std::cout << " TODO - throw an appropriate exception" << std::endl;
        throw;
    }

    for (int i=0; i < request->motor_indice.size(); ++i)
    {
        goToCommands.insert(std::make_pair(request->motor_indice[i],GoToDirCommand((request->drive_direction >= 1 ? Forward : Reverse),request->motor_position[i])));
        profiles.insert(std::make_pair(request->motor_indice[i],ProfileCfg(request->acceleration[i],request->deceleration[i],request->speed[i])));
    }

    // Send the commands
    {
        std::lock_guard<std::mutex> lock(driverMutex_);
        driver_->setProfileCfg(profiles);
        driver_->goToDir(goToCommands);
    }

    // Let's get the positions/speeds
    {
        std::lock_guard<std::mutex> lock(driverMutex_);
        response->positions      = driver_->getPos();
        response->current_speed  = driver_->getSpeed();
    }
}

void
L6470Node::stopCallback(const   std::shared_ptr <l6470_srvs::srv::Stop::Request>  request,
                                std::shared_ptr <l6470_srvs::srv::Stop::Response> response)
{
    if (request == nullptr || response == nullptr)
        std::cout << "TODO" << std::endl;
    std::cout << "STOP callback .. TODO" << std::endl;
}

} // l6470 namespace

#include "class_loader/class_loader_register_macro.h"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.

CLASS_LOADER_REGISTER_CLASS(l6470::L6470Node, rclcpp::Node)
