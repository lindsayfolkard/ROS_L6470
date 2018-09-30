#include "l6470_node.hpp"
#include "l6470_driver/types.h"
#include "l6470_driver/commands.h"
#include "l6470_driver/driverfactory.h"
#include "l6470_driver/simdriver.h"
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

std::string errorToStringInfo(const char *funcInfo, const std::string &detail)
{
    std::stringstream ss;
    ss << funcInfo << " : " << detail;
    return ss.str();
}

}

L6470Node::L6470Node():
    Node("l6470node")
{
    // Instantiate the Stepper Driver from the factory
    const std::string overallCfgFile = "~/dspin_stepper_configs/ros_overall_config.txt"; // TODO - figure out where this file can be stored/deployed nicely
    std::cout << "Try to instantiate stepper motor driver with ros_overall_config file from " << overallCfgFile << std::endl;
    //OverallCfg overallCfg(overallCfgFile);
    driver_ = std::unique_ptr<AbstractDriver>(new SimDriver({StepperMotor(),StepperMotor(),StepperMotor()}));
    //driver_ = factoryMakeDriver(overallCfg);
    assert(driver_ || !"driver is null pointer!" );
    //std::cout << "Instantiated " << overallCfg.controllerType_ << " stepper motor driver with " << overallCfg.cfgFiles_.size() << " daisy chained steppers" << std::endl;

    // Create a publisher of l6470_msgs::msg::Pose msgs
    posePublisher_    = create_publisher<l6470_msgs::msg::MultiPose>("multipose");
    statusPublisher_ = create_publisher<l6470_msgs::msg::MultiStatus>("mutistatus");

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
    std::vector<uint32_t>    speeds;
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

    for (unsigned int motor=0;  motor < positions.size(); ++motor)
    {
        l6470_msgs::msg::Pose state;
        state.position = positions[motor];
        state.speed     = static_cast<int>(speeds[motor]);
        // Timestamp ???
        
        poseMsg->motor_states.push_back(state);
    }
    
    RCLCPP_INFO(this->get_logger(), "Published:  data")

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

    // Hack - Let's check for a rough timeout on manaul speed commands
    const std::chrono::milliseconds timeOutThreshold(500);
    if (lastManualSpeedUpdate_ && ((std::chrono::steady_clock::now() - *lastManualSpeedUpdate_) > timeOutThreshold))
    {
        RCLCPP_WARN(this->get_logger(), "Manual Control Timed Out!");
        lastManualSpeedUpdate_.reset();
        driver_->stopAllSoft();
    }
}

void
L6470Node::manualSpeedCallback(const l6470_msgs::msg::ManualSpeed::UniquePtr manualSpeed)
{  
    if (!manualSpeed.get())
        throw std::invalid_argument(errorToStringInfo(__func__,"ManualSpeed is nullptr"));

    // There is a potential for speed spamming. Let's only update the speed if it has been long enough
    const std::chrono::milliseconds timeThreshold(50);

    if (!lastManualSpeedUpdate_)
        RCLCPP_INFO(this->get_logger(), "Manual Control Enabled");

    if (lastManualSpeedUpdate_ && ((std::chrono::steady_clock::now() - *lastManualSpeedUpdate_) < timeThreshold))
        return;

    lastManualSpeedUpdate_ = std::chrono::steady_clock::now();

    // Create an appropriate run command
    //
    RCLCPP_INFO(this->get_logger(), "Set Manual Speed : ");
    std::map <int,RunCommand> runCommands;
    int motor=0;
    for (const auto &speed : manualSpeed->speed)
    {
        runCommands.insert(std::make_pair(motor,RunCommand((speed > 0 ? Forward : Reverse),static_cast<float>(speed))));
        ++motor;
    }

    std::lock_guard<std::mutex> lock(driverMutex_);

    // Send the command to the motors
    driver_->run(runCommands);
}

void
L6470Node::goToPositionCallback(const   std::shared_ptr <l6470_srvs::srv::GoToPosition::Request>   request,
                                std::shared_ptr <l6470_srvs::srv::GoToPosition::Response> response )
{
    if (!request.get() || !response.get())
        throw std::invalid_argument(errorToStringInfo(__func__ ,"One of arguments is nullptr"));

    // Get the desired positions
    std::map <int,GoToDirCommand> goToDirCommands;
    std::map <int,ProfileCfg>  profiles;

    // Check that the vectors are okay
    // TODO - change the command to be a vector or GoToPositiong to avoid this bullshit
    if (!(request->motor_indice.size() == request->motor_position.size()
          == request->drive_direction.size()
          == request->acceleration.size()
          == request->deceleration.size()
          == request->speed.size()))
    {
        std::cout << " TODO - throw an appropriate exception" << std::endl;
        throw;
    }

    for (size_t i=0; i < request->motor_indice.size(); ++i)
    {
        goToDirCommands.insert(std::make_pair(request->motor_indice[i],GoToDirCommand((request->drive_direction[i] == l6470_srvs::srv::GoToPosition::Request::DRIVE_DIR_FORWARD ? Forward : Reverse),request->motor_position[i])));
        profiles.insert(std::make_pair(request->motor_indice[i],ProfileCfg(static_cast<float>(request->acceleration[i]),static_cast<float>(request->deceleration[i]),static_cast<float>(request->speed[i]))));
    }

    // Send the commands
    {
        std::lock_guard<std::mutex> lock(driverMutex_);
        driver_->setProfileCfg(profiles);
        driver_->goToDir(goToDirCommands);
    }

    // Let's get the positions/speeds
    {
        std::lock_guard<std::mutex> lock(driverMutex_);
        response->positions      = driver_->getPos();
        const auto speedVec = driver_->getSpeed();
        response->current_speed  = std::vector<int>(speedVec.begin(),speedVec.end());
    }
}

void
L6470Node::stopCallback(const   std::shared_ptr <l6470_srvs::srv::Stop::Request>  request,
                        std::shared_ptr <l6470_srvs::srv::Stop::Response> response)
{
    if (!request.get() || !response.get())
        throw std::invalid_argument(errorToStringInfo(__func__, "One of arguments is nullptr"));

    std::lock_guard<std::mutex> lock(driverMutex_);

    if (request->stop_type == static_cast<uint8_t>(l6470_srvs::srv::Stop::Request::STOP_HARD))
    {
        driver_->hardStop(std::vector<int>(request->motor_indice.begin(),request->motor_indice.end()));
    }
    else if (request->stop_type == l6470_srvs::srv::Stop::Request::STOP_HIZ)
    {
        driver_->hardHiZ(std::vector<int>(request->motor_indice.begin(),request->motor_indice.end()));
    }
    else if (request->stop_type == l6470_srvs::srv::Stop::Request::STOP_SOFT)
    {
        driver_->softStop(std::vector<int>(request->motor_indice.begin(),request->motor_indice.end()));
    }
    else
    {
        throw std::invalid_argument(errorToStringInfo(__func__,"Stop Type is invalid (" + std::to_string((int)request->stop_type)));
    }
}

} // l6470 namespace

#include "class_loader/register_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.

CLASS_LOADER_REGISTER_CLASS(l6470::L6470Node, rclcpp::Node)
