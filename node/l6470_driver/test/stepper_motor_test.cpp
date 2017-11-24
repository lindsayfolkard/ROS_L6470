#include <iostream>
#include "../abstractdriver.h"
#include "../powerstepdriver.h"
#include "../motor.h"
#include <stdexcept>
#include <string>
#include <regex>
#include <assert.h>
#include <chrono>

/// @author      : Lindsay Folkard
/// @date        : TODO
/// @description : A simple program to interact with the Stepper driver
///

using namespace std;

//void printMenu();
//void printUsage(int argc, char **argv);
//void handleMenuOption(const std::string &input, AutoDriver &driver);
//std::vector<long> getArguments(const std::string &input,int argCount);

int main (int argc, char ** argv)
{
    // Create the stepper motor
    Stepper_57BYGH51 stepper;
    std::vector<StepperMotor> motors = {stepper,stepper,stepper};

//    // Instantiate the AutoDriver
//    cout << "Try to instantiate the driver" << endl;
//    //MultiDriver driver(motors,0,0,0,CommsDebugEverything); CommsDebugNothing
//    MultiDriver driver(motors,0,0,0,CommsDebugEverything);
//    cout << "Instantiated the driver!" << endl;

//    // Let's try to do some simple shit
//    const auto start = std::chrono::steady_clock::now();
//    std::vector<Status> statusVector = driver.getStatus();
//    const auto end = std::chrono::steady_clock::now();
//    std::chrono::duration<double> diff = end-start;
//    int count=1;
//    for (const Status &status : statusVector)
//    {
//	cout << "Status for motor" << count++ << " is " << status << std::endl << std::endl;
//    }
    
//    cout << "Test setting the motors to softStop" << std::endl;
//    cout << "Status : " << driver.getStatus(0) << " , time = " << diff.count()*1000000 << " microseconds" <<  std::endl;
//    driver.softStop(0);
//    driver.softStop(1);
//    driver.softStop(2);
//    cout << "Status : " << driver.getStatus(0) << std::endl;

//    usleep(3e6);

//    driver.softStop({0,1,2});

//    // Lets get position
//    cout << "Position : " << driver.getPos(0) << std::endl;
//    cout << "Config   : " << driver.getConfig(0) << std::endl;
//    driver.resetPos(0);
//    cout << "Position : " << driver.getPos(0) << std::endl;

//    // Lets set the config
//    cout << "Original Config is " << driver.getConfig(0);
//    Stepper_57BYGH51 nema23BackEmfConfig;
//    VoltageModeCfg backEmfConfig = BackEmfConfigFromStepper(nema23BackEmfConfig,24,2.5);
//    Config driverConfig;
//    driverConfig.backEmfConfig = backEmfConfig;
//    driverConfig.overCurrentThreshold = OCD_TH_3750m;
//    driver.setConfig(driverConfig,0);
//    cout << "Updated Config is " << driver.getConfig(0);
    
//    // Lets set a profile config
//    ProfileCfg profileCfg = driver.getProfileCfg(0);
//    cout << "ProfileCfg" << profileCfg << std::endl;
//    profileCfg.maxSpeed=650;
//    profileCfg.minSpeed=80;
//    profileCfg.acceleration=100;
//    profileCfg.deceleration=100;

//    driver.setProfileCfg(profileCfg,0);
//    cout << "New profile cfg is " << profileCfg << std::endl;

    // Fuck it let's try to move
    /*driver.softStop(0);
    RunCommand runCommand(Forward,400);
    driver.run(runCommand,0);

    const int timeThreshold = 6;
    double tLive = 0.0;
    while (tLive < timeThreshold)
    {
      const auto end = std::chrono::steady_clock::now();
      std::chrono::duration<double> diff = end-start;
      tLive = diff.count();
      cout << "............ T + " << tLive << " s ..............." << std::endl;
      cout << "Position = " << driver.getPos(0) << std::endl;
      cout << "Speed = " << driver.getSpeed(0) << std::endl;
      cout << "Status = " << driver.getStatus(0) << std::endl;
      std::chrono::duration<double> diff2 = std::chrono::steady_clock::now()-end;
      cout << "Data Acquisition time : " << diff2.count()*1000.0 << " ms" << std::endl;
      cout << "......................................" << std::endl << std::endl;

      usleep(20000);

      if (tLive > 3.0)
      {
         driver.hardStop(0);
	 //driver.softHiZ(0);
      }
    }

    driver.softStop(0);
    */
    //    // Read the config back

    //    printMenu();

    //    // Enter Command Loop
    //    while (true)
    //    {
    //        cout <<">>";
    //        std::string input;
    //        cin >> input;
    //        try
    //        {
    //            handleMenuOption(input,driver);
    //        }
    //        catch (std::exception &e)
    //        {
    //            cout << "Caught exception with message " << e.what() << endl;
    //            cout << "Continuing in the hope everything is fine"<<endl;
    //        }
    //    }
}

void printUsage(int argc, char **argv)
{
    assert(argc > 0 && "Invalid argc value <= 0");
    cout << argv[0]; // TODO - as arguments are made
}

void printMenu()
{

  cout << "============================================================" << endl;
  cout << "                   L6470 Command Menu                       " << endl;
  cout << "============================================================" << endl << endl;

  cout << "------------------- Status Commands ------------------------" << endl;
  cout << " busyCheck" << endl;
  cout << " getStatus" << endl;
  cout << "------------------------------------------------------------" << endl << endl;

  cout << "------------------- Config Set Commands --------------------" << endl;
  cout << " setParam(uint8_t param, unsigned long value , int motor)" << endl;
  cout << " setLoSpdOpt(bool enable , int motor)" << endl;
  cout << " configSyncPin(uint8_t pinFunc, uint8_t syncSteps , int motor)" << endl;
  cout << " configStepMode(uint8_t stepMode , int motor)" << endl;
  cout << " setMaxSpeed(float stepsPerSecond , int motor)" << endl;
  cout << " setMinSpeed(float stepsPerSecond , int motor)" << endl;
  cout << " setFullSpeed(float stepsPerSecond , int motor)" << endl;
  cout << " setAcc(float stepsPerSecondPerSecond , int motor)" << endl;
  cout << " setDec(float stepsPerSecondPerSecond , int motor)" << endl;
  cout << " setOCThreshold(uint8_t threshold , int motor)" << endl;
  cout << " setPWMFreq(int divisor, int multiplier , int motor)" << endl;
  cout << " setSlewRate(int slewRate , int motor)" << endl;
  cout << " setOCShutdown(int OCShutdown , int motor)" << endl;
  cout << " setVoltageComp(int vsCompMode , int motor)" << endl;
  cout << " setSwitchMode(int switchMode , int motor)" << endl;
  cout << " setOscMode(int oscillatorMode , int motor)" << endl;
  cout << " setAccKVAL(uint8_t kvalInput , int motor)" << endl;
  cout << " setDecKVAL(uint8_t kvalInput , int motor)" << endl;
  cout << " setRunKVAL(uint8_t kvalInput , int motor)" << endl;
  cout << " setHoldKVAL(uint8_t kvalInput , int motor)" << endl;
  cout << "------------------------------------------------------------" << endl << endl;

  cout << "------------------- Config Get Commands --------------------" << endl;
  cout << " getParam(uint8_t param , int motor)" << endl;
  cout << " getLoSpdOpt(int motor)" << endl;
  cout << " getStepMode(int motor)" << endl;
  cout << " getMaxSpeed(int motor)" << endl;
  cout << " getMinSpeed(int motor)" << endl;
  cout << " getFullSpeed(int motor)" << endl;
  cout << " getAcc(int motor)" << endl;
  cout << " getDec(int motor)" << endl;
  cout << " getOCThreshold(int motor)" << endl;
  cout << " getPWMFreqDivisor(int motor)" << endl;
  cout << " getPWMFreqMultiplier(int motor)" << endl;
  cout << " getSlewRate(int motor)" << endl;
  cout << " getOCShutdown(int motor)" << endl;
  cout << " getVoltageComp(int motor)" << endl;
  cout << " getSwitchMode(int motor)" << endl;
  cout << " getOscMode(int motor)" << endl;
  cout << " getAccKVAL(int motor)" << endl;
  cout << " getDecKVAL(int motor)" << endl;
  cout << " getRunKVAL(int motor)" << endl;
  cout << " getHoldKVAL(int motor)" << endl;
  cout << "------------------------------------------------------------" << endl << endl;

  cout << "------------------- Operational Commands -------------------" << endl;
  cout << " getPos(int motor)" << endl;
  cout << " getMark(int motor)" << endl;
  cout << " run(uint8_t dir, float stepsPerSec , int motor)" << endl;
  cout << " stepClock(uint8_t dir , int motor)" << endl;
  cout << " move(uint8_t dir, unsigned long numSteps , int motor)" << endl;
  cout << " goTo(long pos , int motor)" << endl;
  cout << " goToDir(uint8_t dir, long pos , int motor)" << endl;
  cout << " goUntil(uint8_t action, uint8_t dir, float stepsPerSec , int motor)" << endl;
  cout << " releaseSw(uint8_t action, uint8_t dir , int motor)" << endl;
  cout << " goHome(int motor)" << endl;
  cout << " goMark(int motor)" << endl;
  cout << " setMark(long newMark , int motor)" << endl;
  cout << " setPos(long newPos , int motor)" << endl;
  cout << " resetPos(int motor)" << endl;
  cout << " resetDev(int motor)" << endl;
  cout << " softStop(int motor)" << endl;
  cout << " hardStop(int motor)" << endl;
  cout << " softHiZ(int motor)" << endl;
  cout << " hardHiZ(int motor)" << endl;
  cout << "------------------------------------------------------------" << endl << endl;

  cout << "============================================================" << endl;
}

//// Really ugly, but hey it works and easy to understand what is going on
//void handleMenuOption(const std::string &input , AutoDriver &driver)
//{

//    if      ( input.find("busyCheck")!= std::string::npos)
//    {
//	cout << "BusyState = " << driver.isBusy() << endl;
//    }
//    else if ( input.find("getStatus")!= std::string::npos)
//    {
//        cout << "Status = " << driver.getStatus() << endl;
//    }
//    else if ( input.find("setParam")!= std::string::npos )
//    {
//        std::vector<long> arguments = getArguments(input,2);
//	driver.setParam(static_cast<ParamRegister> (arguments[0]),arguments[1]);
//    }
//    else if ( input.find("setLoSpdOpt")!= std::string::npos )
//    {
//        assert(!"TODO");
//    }
//    else if ( input.find("configSyncPin")!= std::string::npos )
//    {
//        assert(!"TODO");
//    }
//    else if ( input.find("configStepMode")!= std::string::npos )
//    {
//        std::vector<long> arguments = getArguments(input,1);
//        driver.setStepMode(static_cast<StepMode>(arguments[0]));
//    }
//    else if ( input.find("setMaxSpeed")!= std::string::npos )
//    {
//        std::vector<long> arguments = getArguments(input,1);
//        driver.setMaxSpeed((uint8_t)arguments[0]);
//    }
//    else if ( input.find("setMinSpeed")!= std::string::npos )
//    {
//        std::vector<long> arguments = getArguments(input,1);
//        driver.setMinSpeed((uint8_t)arguments[0]);
//    }
//    else if ( input.find("setFullSpeed")!= std::string::npos )
//    {
//        std::vector<long> arguments = getArguments(input,1);
//        driver.setFullSpeed((uint8_t)arguments[0]);
//    }
//    else if ( input.find("setAcc")!= std::string::npos )
//    {
//        std::vector<long> arguments = getArguments(input,1);
//        driver.setAcc((uint8_t)arguments[0]);
//    }
//    else if ( input.find("setDec")!= std::string::npos )
//    {
//        std::vector<long> arguments = getArguments(input,1);
//        driver.setDec((uint8_t)arguments[0]);
//    }
//    else if ( input.find("setOCThreshold")!= std::string::npos )
//    {
//        assert (!"TODO");
//    }
//    else if ( input.find( "setPWMFreq")!= std::string::npos )
//    {
//        assert (!"TODO");
//    }
//    else if ( input.find("setSlewRate")!= std::string::npos )
//    {
//        assert (!"TODO");
//    }
//    else if ( input.find("setOCShutdown")!= std::string::npos )
//    {
//        assert (!"TODO");
//    }
//    else if ( input.find("setVoltageComp")!= std::string::npos )
//    {
//        assert (!"TODO");
//    }
//    else if ( input.find("setSwitchMode")!= std::string::npos )
//    {
//        assert (!"TODO");
//    }
//    else if ( input.find("setOscMode")!= std::string::npos )
//    {
//        assert (!"TODO");
//    }
//    else if ( input.find("setAccKVAL")!= std::string::npos )
//    {
//        assert (!"TODO");
//    }
//    else if ( input.find("setDecKVAL")!= std::string::npos )
//    {
//        assert (!"TODO");
//    }
//    else if ( input.find("setRunKVAL")!= std::string::npos )
//    {
//        assert (!"TODO");
//    }
//    else if ( input.find("setHoldKVAL")!= std::string::npos )
//    {
//        assert (!"TODO");
//    }
//    else if ( input.find("getParam")!= std::string::npos )
//    {
//        std::vector<long> arguments = getArguments(input,1);
//	cout << "Param " << arguments[0] << " = " << driver.getParam(static_cast<ParamRegister>(arguments[0])) << endl;
//    }
//    else if ( input.find("getLoSpdOpt")!= std::string::npos )
//    {
//        assert (!"TODO");
//    }
//    else if ( input.find("getStepMode")!= std::string::npos )
//    {
//        cout << "Step Mode = " << driver.getStepMode() << endl;
//    }
//    else if ( input.find("getMaxSpeed")!= std::string::npos )
//    {
//        cout << "Max Speed = " << driver.getMaxSpeed() << endl;
//    }
//    else if ( input.find("getMinSpeed")!= std::string::npos )
//    {
//        std::vector<long> arguments = getArguments(input,1);
//        cout << "Min Speed = " << driver.getMinSpeed() << endl;
//    }
//    else if ( input.find("getFullSpeed")!= std::string::npos )
//    {
//        cout << "Full Speed = " << driver.getFullSpeed() << endl;
//    }
//    else if ( input.find("getAcc")!= std::string::npos )
//    {
//        cout << "Acc = " << driver.getAcc() << endl;
//    }
//    else if ( input.find("getDec")!= std::string::npos )
//    {
//        cout << "Dec = " << driver.getDec() << endl;
//    }
//    else if ( input.find("getOCThreshold(!= std::string::npos )")!= std::string::npos )
//    {
//        assert (!"TODO");
//    }
//    else if ( input.find("getPWMFreqDivisor(!= std::string::npos )")!= std::string::npos )
//    {
//        assert (!"TODO");
//    }
//    else if ( input.find("getPWMFreqMultiplier(!= std::string::npos )")!= std::string::npos )
//    {
//        assert (!"TODO");
//    }
//    else if ( input.find("getSlewRate(!= std::string::npos )")!= std::string::npos )
//    {
//        assert (!"TODO");
//    }
//    else if ( input.find("getOCShutdown(!= std::string::npos )")!= std::string::npos )
//    {
//        assert (!"TODO");
//    }
//    else if ( input.find("getVoltageComp(!= std::string::npos )")!= std::string::npos )
//    {
//        assert (!"TODO");
//    }
//    else if ( input.find("getSwitchMode(!= std::string::npos )")!= std::string::npos )
//    {
//        assert (!"TODO");
//    }
//    else if ( input.find("getOscMode(!= std::string::npos )")!= std::string::npos )
//    {
//        assert (!"TODO");
//    }
//    else if ( input.find("getAccKVAL(!= std::string::npos )")!= std::string::npos )
//    {
//        assert (!"TODO");
//    }
//    else if ( input.find("getDecKVAL(!= std::string::npos )")!= std::string::npos )
//    {
//        assert (!"TODO");
//    }
//    else if ( input.find("getRunKVAL(!= std::string::npos )")!= std::string::npos )
//    {
//        assert (!"TODO");
//    }
//    else if ( input.find("getHoldKVAL(!= std::string::npos )")!= std::string::npos )
//    {
//        assert (!"TODO");
//    }
//    else if ( input.find("getPos")!= std::string::npos )
//    {
//        cout << "Pos = " << driver.getPos() << endl;
//    }
//    else if ( input.find("getMark")!= std::string::npos )
//    {
//        cout << "Mark = " << driver.getMark() << endl;
//    }
//    else if ( input.find("run")!= std::string::npos )
//    {
//        std::vector<long> arguments = getArguments(input,2);
//	driver.run(static_cast<MotorSpinDirection>(arguments[0]),arguments[1]);
//    }
//    else if ( input.find("stepClock")!= std::string::npos )
//    {
//        std::vector<long> arguments = getArguments(input,1);
//	driver.stepClock(static_cast<MotorSpinDirection>(arguments[0]));
//    }
//    else if ( input.find("move")!= std::string::npos )
//    {
//        std::vector<long> arguments = getArguments(input,2);
//	driver.move(static_cast<MotorSpinDirection>(arguments[0]),arguments[1]);
//    }
//    else if ( input.find("goTo")!= std::string::npos )
//    {
//        std::vector<long> arguments = getArguments(input,1);
//        driver.goTo(arguments[0]);
//    }
//    else if ( input.find("goToDir")!= std::string::npos )
//    {
//        std::vector<long> arguments = getArguments(input,2);
//	driver.goToDir(static_cast<MotorSpinDirection>(arguments[0]),arguments[1]);
//    }
//    else if ( input.find("goUntil")!= std::string::npos )
//    {
//        std::vector<long> arguments = getArguments(input,3);
//	driver.goUntil(static_cast<MotorSpinDirection>(arguments[0]),arguments[1],static_cast<Action>(arguments[2]));
//    }
//    else if ( input.find("releaseSw")!= std::string::npos )
//    {
//        std::vector<long> arguments = getArguments(input,2);
//	driver.releaseSw(static_cast<MotorSpinDirection>(arguments[0]),static_cast<Action>(arguments[1]));
//    }
//    else if ( input.find("goHome")!= std::string::npos )
//    {
//        driver.goHome();
//    }
//    else if ( input.find("goMark")!= std::string::npos )
//    {
//        driver.goMark();
//    }
//    else if ( input.find("setMark")!= std::string::npos )
//    {
//        std::vector<long> arguments = getArguments(input,1);
//        driver.setMark(arguments[0]);
//    }
//    else if ( input.find("setPos")!= std::string::npos )
//    {
//        std::vector<long> arguments = getArguments(input,1);
//        driver.setPos(arguments[0]);
//    }
//    else if ( input.find("resetPos")!= std::string::npos )
//    {
//        driver.resetPos();
//    }
//    else if ( input.find("resetDev")!= std::string::npos )
//    {
//        driver.resetDev();
//    }
//    else if ( input.find("soft")!= std::string::npos )
//    {
//        driver.softStop();
//    }
//    else if ( input.find("hardStop")!= std::string::npos )
//    {
//        driver.hardStop();
//    }
//    else if ( input.find("softHiZ")!= std::string::npos )
//    {
//        driver.softHiZ();
//    }
//    else if ( input.find("hardHiZ")!= std::string::npos )
//    {
//        driver.hardHiZ();
//    }
//    else
//    {
//        throw std::runtime_error("Unable to parse input command " + input);
//    }

//}

std::vector<long> getArguments(const std::string &input,int argCount)
{
    // Find the brackets
    //    const std::regex bracketRegex("([0-9,]*)");
    //    std::smatch pieces_match;
    //    std::regex_match(input,pieces_match,bracketRegex);

    // Split the matching string by the comma marks

    //
    return std::vector<long> ();

}
