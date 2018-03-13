#include <iostream>
#include "../abstractdriver.h"
#include "../powerstepdriver.h"
#include "../motor.h"
#include <stdexcept>
#include <string>
#include <regex>
#include <assert.h>
#include <chrono>
#include <functional>
#include "../config.h"
#include <utility>

// Test Profile Config
void testSetProfileCfg(BaseDriver &baseDriver, std::string &testName, bool debugEnabled = false);

// Speed Commands
void testRun(BaseDriver &baseDriver, std::string &testName, bool debugEnabled = false);

// Intentionally not auto-testing (have no easy way of triggering a switch easily)
//void goUntil(const std::map<int, DataCommand> &goUntilCommands) override;

// Position Commands
void testMove   (BaseDriver &baseDriver, std::string &testName, bool debugEnabled = false);
void testGoTo   (BaseDriver &baseDriver, std::string &testName, bool debugEnabled = false);
void testGoToDir(BaseDriver &baseDriver, std::string &testName, bool debugEnabled = false);
void testGoHome (BaseDriver &baseDriver, std::string &testName, bool debugEnabled = false);
void testGoMark (BaseDriver &baseDriver, std::string &testName, bool debugEnabled = false);

// Stop Commands
void testSoftStop (BaseDriver &baseDriver, std::string &testName, bool debugEnabled = false);
void testHardStop (BaseDriver &baseDriver, std::string &testName, bool debugEnabled = false);
void testSoftHiZ  (BaseDriver &baseDriver, std::string &testName, bool debugEnabled = false);
void testHardHiZ  (BaseDriver &baseDriver, std::string &testName, bool debugEnabled = false);

// Set Commands
void testSetPos  (BaseDriver &baseDriver, std::string &testName, bool debugEnabled = false);
void testResetPos(BaseDriver &baseDriver, std::string &testName, bool debugEnabled = false);
void testResetDev(BaseDriver &baseDriver, std::string &testName, bool debugEnabled = false);

class TestFailException : public std::exception
{
public:
    TestFailException (const std::string &failString):failString_(failString){}
    virtual char const * what() const noexcept override { return failString_.c_str(); }
private:
    const std::string failString_;
};

bool isNear(float a, float b , float tolerance);

int main(int argc, char **argv)
{
    // Instantiate the driver
    // My test rig has two motors with the following
    Stepper_42BYGHW811 nema17Stepper;
    Stepper_57BYGH51   nema23Stepper;
    std::vector<StepperMotor> motors = {nema17Stepper,nema23Stepper};

    // Let's try to make a PowerStepConfig from motors
    CurrentModeCfg currentModeCfg;
    CommonConfig   commonConfig;
    //commonConfig.overCurrentThreshold = OCD_TH_4500m;
    PowerStepCfg powerStepConfig17(commonConfig,currentModeCfg,BackEmfConfigFromStepper(nema17Stepper));
    PowerStepCfg powerStepConfig23(commonConfig,currentModeCfg,BackEmfConfigFromStepper(nema23Stepper));

    std::vector<PowerStepCfg> cfgs = {powerStepConfig23,powerStepConfig17};

    // Instantiate the PowerStep Driver
    std::cout << "Instantiate the driver" << std::endl;
    PowerStepDriver driver(motors,cfgs,0,CommsDebugNothing);
    std::cout << "Driver successfully instantiated" << std::endl;

    const bool debugEnabled = false;

    // Make the test cases
    std::vector<std::function<void(BaseDriver &baseDriver, std::string &testName, bool debugEnabled)>> testFunctions = {
        testSetProfileCfg,
        testRun,
        testMove,
        testGoTo,
        testGoToDir,
        testGoHome,
        testGoMark,
        testSoftStop,
        testHardStop,
        testSoftHiZ,
        testHardHiZ,
        testSetPos,
        testResetPos,
        testResetDev
    };

    // Run the tests
    int testNumber=1;
    int failCount=0;
    for (auto &test : testFunctions)
    {
        std::string testName;
        try
        {
            test(driver,testName,debugEnabled);
        }
        catch (std::exception &e)
        {
            std::cout << "Test " << testNumber <<  " - " << testName << " : " << addColour("Failed!",Red)
                      << std::endl << " Reason : " << e.what() << std::endl;
            ++failCount;
        }

        // Print out the result if successful
        std::cout << "Test " << testNumber << " - " << testName << " : " << addColour("Passed!",Green) << std::endl;
        ++testNumber;
    }

    // Print out the aggregate result
    if ( failCount != 0 )
        std::cout << "Overall Result : " << addColour("Failed",Red) << std::endl;
    else
        std::cout << "Overall Result : " << addColour("Passed",Green) << std::endl;

}

bool
isNear(float a, float b , float tolerance)
{
    return ( std::abs(a-b) <= tolerance);
}

// Test Profile Config
void
testSetProfileCfg (BaseDriver &baseDriver, std::string &testName, bool debugEnabled)
{
    testName = "SetProfileConfig";

    // Create a reasonable profile
    std::map<int,ProfileCfg> profileCfgMap;
    for (unsigned int i=0; i < baseDriver.motors_.size() ; ++i)
    {
        ProfileCfg profile;
        profile.acceleration= 100 + (i*50);
        profile.deceleration= 100 + (i*50);
        profile.maxSpeed= 550 + (i*50);
        profile.minSpeed= 50 + (i*50);
        profileCfgMap.insert(std::make_pair(i,profile));

        if (debugEnabled)
            std::cout << "Set motor " << i << " profileconfig to be : " << std::endl << profile << std::endl;
    }

    // Set the new config
    baseDriver.setProfileCfg(profileCfgMap);

    // read the profile configs back and confirm everything ok
    for (unsigned int i=0; i < profileCfgMap.size() ; ++i)
    {
        // Check that the config is as expected
        ProfileCfg readBackCfg = baseDriver.getProfileCfg(i);

        if (readBackCfg != profileCfgMap[i])
        {
            std::stringstream ss;
            ss << "ProfileConfig set to motor " << i << " doe not match config read back. Hence : "
               << "Set Profile Config" << std::endl
               << readBackCfg << std::endl << std::endl
               << "Read Profile Config" << std::endl
               << profileCfgMap[i];
            throw TestFailException(ss.str());
        }
    }
}

// Speed Commands
void
testRun (BaseDriver &baseDriver, std::string &testName, bool debugEnabled)
{
    testName = "Run";

    // Make sure the motors are stopped
    baseDriver.stopAllSoft();

    std::map<int,RunCommand> commandMap;

    float stepsPerSecond=300;
    for (int i=0 ; i < baseDriver.motors_.size(); ++i)
    {
        commandMap.insert(std::pair<int,RunCommand>(i,RunCommand((i%2 == 0 ? Reverse : Forward),stepsPerSecond)));
        stepsPerSecond+=30;
    }

    // Set the motor to run
    baseDriver.run(commandMap);

    // Wait for the motors to spin up a bit
    sleep(2);

    // Check motors are running at the right speed
    for (auto &element : commandMap)
    {
        const RunCommand cmd = element.second;

        Status status = baseDriver.getStatus()[element.first];
        float  speed  = baseDriver.getSpeed(element.first);

        if (status.spinDirection != cmd.direction)
        {
            throw TestFailException("Spin direction for motor " + std::to_string(element.first) + "("
                                    + toString(status.spinDirection) + ") does not match expected direction (" +
                                    toString(cmd.direction) + ")");
        }

        if (!isNear(speed,cmd.stepsPerSec,5.0))
        {
            throw TestFailException("Speed for motor " + std::to_string(element.first) + "(" +
                                    std::to_string(speed) + " steps/s) does not match the expected speed ("
                                    + std::to_string(cmd.stepsPerSec) + ")");
        }
    }

    baseDriver.stopAllSoft();
    sleep(3); // give them some time to stop
}

// Intentionally not auto-testing (have no easy way of triggering a switch easily)
//void goUntil(const std::map<int, DataCommand> &goUntilCommands) override;

// Position Commands
void
testMove (BaseDriver &baseDriver, std::string &testName, bool debugEnabled)
{
    testName = "Move";

    // Stop all the motors and zero position before doing anything
    baseDriver.stopAllHard();
    baseDriver.setAllPos(0);
    baseDriver.clearStatus();

    // Create the move commands and set each stepper to be at the 0 pos
    std::map <int,MoveCommand> cmdMap;
    const int minPos=300;
    const int posIncrement=50;
    for (int i=0 ; i < baseDriver.motors_.size(); ++i)
    {
        baseDriver.setPos(0,i);
        MotorSpinDirection spinDir = (i%2 == 0 ? Forward : Reverse);
        int position = (spinDir == Forward ? 1 : -1) * (minPos + (i*posIncrement));
        MoveCommand moveCommand(spinDir,position);
        cmdMap.insert(std::pair<int,MoveCommand>(i,moveCommand));
    }

    // Execute the move command
    baseDriver.move(cmdMap);

    // Wait for motors to reach their position roughly
    // assume they have well and truly reached the desired position by now...
    sleep(5);

    // Check that we have reached the correct positions
    for (auto &element : cmdMap)
    {
        MoveCommand cmd  = element.second;
        double motorPosition = baseDriver.getPos(element.first);

        if (motorPosition != ((cmd.numSteps)*(cmd.direction == Forward ? 1 : -1)))
        {
            throw TestFailException("Motor " + std::to_string(element.first)
                                    + " failed to reach the desired position (pos = " + std::to_string(cmd.numSteps)
                                    + " steps). Current position is " + std::to_string(motorPosition) + ")");
        }
    }

    // Stop all motors again as sanity check
    baseDriver.stopAllHard();
    sleep(2);
}

void
testGoTo (BaseDriver &baseDriver, std::string &testName, bool debugEnabled)
{
    testName = "GoTo";

    baseDriver.stopAllHard();
    baseDriver.setAllPos(0);
    baseDriver.clearStatus();

    std::map <int,GoToCommand> cmdMap;
    const int minPos=300;
    const int posIncrement=50;
    for (int i=0 ; i < baseDriver.motors_.size(); ++i)
    {
        baseDriver.setPos(0,i);
        int position = minPos + (i*posIncrement);
        GoToCommand goToCommand(position);
        cmdMap.insert(std::pair<int,GoToCommand>(i,goToCommand));
    }

    // execute the goto command
    baseDriver.goTo(cmdMap);

    // Wait for motors to reach their position roughly
    // assume they have well and truly reached the desired position by now...
    sleep(5);

    // Check that we have reached the correct positions
    for (auto &element : cmdMap)
    {
        GoToCommand cmd  = element.second;
        double motorPosition = baseDriver.getPos(element.first);

        if (motorPosition != cmd.pos)
        {
            throw TestFailException("Motor " + std::to_string(element.first)
                                    + " failed to reach the desired position (pos = " + std::to_string(cmd.pos)
                                    + " steps). Current position is " + std::to_string(motorPosition) + ")");
        }
    }

    // Stop all motors again as sanity check
    baseDriver.stopAllHard();
    sleep(2);
}

void
testGoToDir(BaseDriver &baseDriver, std::string &testName, bool debugEnabled)
{
    testName = "GoToDir";

    baseDriver.stopAllHard();
    baseDriver.setAllPos(0);
    baseDriver.clearStatus();

    std::map <int,GoToDirCommand> cmdMap;
    const int minPos=300;
    const int posIncrement=50;
    for (int i=0 ; i < baseDriver.motors_.size(); ++i)
    {
        baseDriver.setPos(0,i);
        MotorSpinDirection spinDir = (i%2 == 0 ? Forward : Reverse);
        int position = (spinDir == Forward ? 1 : -1) * (minPos + (i*posIncrement));
        GoToDirCommand goToDirCommand(spinDir,position);
        cmdMap.insert(std::pair<int,GoToDirCommand>(i,goToDirCommand));
    }

    // Execute the command
    baseDriver.goToDir(cmdMap);

    // Wait for motors to reach their position roughly
    // assume they have well and truly reached the desired position by now...
    sleep(5);

    // Check that we have reached the correct positions
    for (auto &element : cmdMap)
    {
        GoToDirCommand cmd  = element.second;
        double motorPosition = baseDriver.getPos(element.first);

        if (motorPosition != cmd.pos)
        {
            throw TestFailException("Motor " + std::to_string(element.first)
                                    + " failed to reach the desired position (pos = " + std::to_string(cmd.pos)
                                    + " steps). Current position is " + std::to_string(motorPosition) + ")");
        }
    }

    // Stop all motors again as sanity check
    baseDriver.stopAllHard();
    sleep(2);
}

void
testGoHome (BaseDriver &baseDriver, std::string &testName, bool debugEnabled)
{
    testName = "GoHome";

    baseDriver.stopAllHard();
    baseDriver.setAllPos(0);
    baseDriver.clearStatus();

    // Create the move commands and set each stepper to be at the 0 pos
    std::vector<int> motors;
    for (int i=0 ; i < baseDriver.motors_.size(); ++i)
    {
        motors.push_back(i);
    }

    // Send the command
    baseDriver.goHome(motors);

    // Wait for motors to reach their position roughly
    // assume they have well and truly reached the desired position by now...
    sleep(5);

    // Check that we have reached the correct positions
    for (const auto &motor : motors)
    {
        double motorPosition = baseDriver.getPos(motor);

        if (motorPosition != 0)
        {
            throw TestFailException("Motor " + std::to_string(motor)
                                    + " failed to reach the desired home position (pos = 0"
                                    + " steps). Current position is " + std::to_string(motorPosition) + ")");
        }
    }

    // Stop all motors again as sanity check
    baseDriver.stopAllHard();
    sleep(2);
}

void
testGoMark (BaseDriver &baseDriver, std::string &testName, bool debugEnabled)
{
    testName = "GoMark";

    // Let's create a few mark positions
    baseDriver.stopAllHard();
    baseDriver.setAllPos(0);

    int posIncrement = 50;
    const int initialPos=200;
    std::vector<int> motors;
    for (unsigned int i=0; i < baseDriver.motors_.size() ; ++i)
    {
        baseDriver.setMark(initialPos + (i*posIncrement),i);
        motors.push_back(i);
    }

    // Let's go to mark
    baseDriver.goMark(motors);

    // Wait for a bit
    sleep(4);

    // set the new positions as mark
    for (unsigned int i=0; i < baseDriver.motors_.size(); ++i)
    {
        // get the mark
        int markPos = baseDriver.getMark()[i];
        int motorPosition = baseDriver.getPos(i);
        if (motorPosition != markPos)
        {
            throw TestFailException("Motor " + std::to_string(i) +
                                    " failed to reach the desired mark position (pos = " + std::to_string(markPos)
                                    + " steps). Current position is " + std::to_string(motorPosition) + ")");

        }
    }

    baseDriver.stopAllSoft();
}

//void testStop(BaseDriver &baseDriver,
//              std::function <void(BaseDriver,int)> &stopFunction,
//              std::function <void(BaseDriver,int)> &testFunction)
//{

//}


// Stop Commands
void
testSoftStop (BaseDriver &baseDriver, std::string &testName, bool debugEnabled)
{
    testName = "SoftStop";

    // Get the motors spinning
    const int stepsPerSecond=400;
    for (int i=0; i < baseDriver.motors_.size(); ++i)
    {
        RunCommand runCommand(Forward,stepsPerSecond);
        baseDriver.run(runCommand,i);
    }

    // Wait for all the motors to be spinning
    sleep(2);

    // Check the status - TODO if needed

    // Soft Stop each Motor and check the status
    for (int i=0; i < baseDriver.motors_.size(); ++i)
    {
        baseDriver.softStop(i);
        const int waitTimeoutS = 5;
        const int sleepTimeMs = 100;
        int waitCount=0;
        const int maxWaitCount = (waitTimeoutS*1e3)/sleepTimeMs;

        while (waitCount < maxWaitCount && baseDriver.getStatus(i).motorStatus != STATUS_MOT_STATUS_STOPPED)
        {
            ++waitCount;
            usleep(sleepTimeMs*1e3);
        }

        if (waitCount >= maxWaitCount)
            throw TestFailException("Motor " + std::to_string(i) + " does not appeared to have come to a " + testName);
    }
}

void
testHardStop (BaseDriver &baseDriver, std::string &testName, bool debugEnabled)
{
    testName = "HardStop";
}

void
testSoftHiZ (BaseDriver &baseDriver, std::string &testName, bool debugEnabled)
{
    testName = "SoftHiZ";
}

void
testHardHiZ (BaseDriver &baseDriver, std::string &testName, bool debugEnabled)
{
    testName = "HardHiZ";
}

void
testSetPos (BaseDriver &baseDriver, std::string &testName, bool debugEnabled)
{
    testName = "SetPos";
}

void
testResetPos(BaseDriver &baseDriver, std::string &testName, bool debugEnabled)
{
    testName = "ResetPos";
}

void
testResetDev(BaseDriver &baseDriver, std::string &testName, bool debugEnabled)
{
    testName = "ResetDev";
}
