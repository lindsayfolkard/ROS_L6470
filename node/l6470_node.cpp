#include "l6470_node.hpp"
#include "l6470_driver/types.h"
#include "l6470_driver/commands.h"
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

L6470Node::L6470Node():
    Node("l6470node"),
    count_(0)
{
    // ??? leave this for the init stage --> yes , this should be in the init stage)
    std::cout << "Creating L6470 Node!"<<std::endl;

    // Create a publisher of l6470_msgs::msg::Pose msgs
    posePublisher_ = create_publisher<l6470_msgs::msg::MultiPose>("multipose");

    // Use a timer to schedule periodic message publishing.
    std::chrono::milliseconds period(500);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&L6470Node::on_timer, this));

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

    // Fuck it Im lazy and don't want to think too hard about how to configure the boards
    //const std::string masterConfig = "/home/lindsay/stepper_config";
    const std::vector <std::string>stepperConfigs = {"motor1.cfg","motor2.cfg","motor3.cfg"};
    const std::string filePath = "/home/lindsay/git/src/ROS_L6470/node/configs/";

    // Create the config vector
    const int vBus = 24;
    const double ratedCurrent=3.75;
    
    std::vector<StepperMotor> motors;
    std::vector<Config>       cfgs;    

    for (const auto &cfgFile : stepperConfigs)
    {
        // Open the Config
        std::ifstream inFile;
	inFile.open(filePath+cfgFile,std::ifstream::in);
        if (!inFile.is_open())
	{
           std::cout << "Unable to open cfg file " << filePath+cfgFile << std::endl;
	   continue;
	}
        std::string str((std::istreambuf_iterator<char>(inFile)),
                         std::istreambuf_iterator<char>());
	std::cout << "Opened " << filePath+cfgFile << " with data --> " << str << std::endl;	

        // Try to parse the Config
        Config config = cfgFromString(str);
        
        // Parse the motor config
        StepperMotor motorCfg = stepperFromString(str);
        BackEmfConfig backEmfCfg = BackEmfConfigFromStepper(motorCfg,vBus,ratedCurrent);
        config.backEmfConfig = backEmfCfg;

        // Add to the vector
        motors.push_back(motorCfg);
        cfgs.push_back(config);

	// Let' print out ot make rusre vereyjhting makes sense
        std::cout << "Motor " << motorCfg << std::endl;
        std::cout << "Config " << cfgFile << " : " << config << std::endl;
    }
    
    // Let's instantiate the driver
    const int chipSelectPin = 0; // huh
    const int resetPin = 0; // Is it even needed
    //driver_.reset(new MultiDriver(motors,configs,chipSelectPin,resetPin));
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

//#include <class_loader/class_loader_register_macro.h>

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.

//CLASS_LOADER_REGISTER_CLASS(l6470::L6470Node, rclcpp::Node)
