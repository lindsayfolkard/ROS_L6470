#include <mraa.hpp>
#include <iostream>
#include "driver.h"
#include <stdexcept>
#include <string>
#include <regex>
#include <assert.h>

/// @author      : Lindsay Folkard
/// @date        : TODO
/// @description : A simple program to interact with the SparkFunAutodriver tweaked to user mraa.
///

using namespace std;

void printMenu();
void printUsage(int argc, char **argv);
void handleMenuOption(const std::string &input, AutoDriver &driver);
std::vector<long> getArguments(const std::string &input,int argCount);


int main (int argc, char ** argv)
{
    // Instantiate the AutoDriver
    AutoDriver driver(0,0,0);

    // Let's try to do some simple shit
    cout << "Status : " << driver.getStatus() << std::endl;
    driver.softHiZ();
    cout << "Status : " << driver.getStatus() << std::endl;

    // Lets get position
    cout << "Position : " << driver.getPos() << std::endl;
    cout << "Config   : " << driver.getConfig() << std::endl;
    driver.resetPos();
    cout << "Position : " << driver.getPos() << std::endl;

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
  cout << " setParam(uint8_t param, unsigned long value)" << endl;
  cout << " setLoSpdOpt(bool enable)" << endl;
  cout << " configSyncPin(uint8_t pinFunc, uint8_t syncSteps)" << endl;
  cout << " configStepMode(uint8_t stepMode)" << endl;
  cout << " setMaxSpeed(float stepsPerSecond)" << endl;
  cout << " setMinSpeed(float stepsPerSecond)" << endl;
  cout << " setFullSpeed(float stepsPerSecond)" << endl;
  cout << " setAcc(float stepsPerSecondPerSecond)" << endl;
  cout << " setDec(float stepsPerSecondPerSecond)" << endl;
  cout << " setOCThreshold(uint8_t threshold)" << endl;
  cout << " setPWMFreq(int divisor, int multiplier)" << endl;
  cout << " setSlewRate(int slewRate)" << endl;
  cout << " setOCShutdown(int OCShutdown)" << endl;
  cout << " setVoltageComp(int vsCompMode)" << endl;
  cout << " setSwitchMode(int switchMode)" << endl;
  cout << " setOscMode(int oscillatorMode)" << endl;
  cout << " setAccKVAL(uint8_t kvalInput)" << endl;
  cout << " setDecKVAL(uint8_t kvalInput)" << endl;
  cout << " setRunKVAL(uint8_t kvalInput)" << endl;
  cout << " setHoldKVAL(uint8_t kvalInput)" << endl;
  cout << "------------------------------------------------------------" << endl << endl;

  cout << "------------------- Config Get Commands --------------------" << endl;
  cout << " getParam(uint8_t param)" << endl;
  cout << " getLoSpdOpt()" << endl;
  cout << " getStepMode()" << endl;
  cout << " getMaxSpeed()" << endl;
  cout << " getMinSpeed()" << endl;
  cout << " getFullSpeed()" << endl;
  cout << " getAcc()" << endl;
  cout << " getDec()" << endl;
  cout << " getOCThreshold()" << endl;
  cout << " getPWMFreqDivisor()" << endl;
  cout << " getPWMFreqMultiplier()" << endl;
  cout << " getSlewRate()" << endl;
  cout << " getOCShutdown()" << endl;
  cout << " getVoltageComp()" << endl;
  cout << " getSwitchMode()" << endl;
  cout << " getOscMode()" << endl;
  cout << " getAccKVAL()" << endl;
  cout << " getDecKVAL()" << endl;
  cout << " getRunKVAL()" << endl;
  cout << " getHoldKVAL()" << endl;
  cout << "------------------------------------------------------------" << endl << endl;

  cout << "------------------- Operational Commands -------------------" << endl;
  cout << " getPos()" << endl;
  cout << " getMark()" << endl;
  cout << " run(uint8_t dir, float stepsPerSec)" << endl;
  cout << " stepClock(uint8_t dir)" << endl;
  cout << " move(uint8_t dir, unsigned long numSteps)" << endl;
  cout << " goTo(long pos)" << endl;
  cout << " goToDir(uint8_t dir, long pos)" << endl;
  cout << " goUntil(uint8_t action, uint8_t dir, float stepsPerSec)" << endl;
  cout << " releaseSw(uint8_t action, uint8_t dir)" << endl;
  cout << " goHome()" << endl;
  cout << " goMark()" << endl;
  cout << " setMark(long newMark)" << endl;
  cout << " setPos(long newPos)" << endl;
  cout << " resetPos()" << endl;
  cout << " resetDev()" << endl;
  cout << " softStop()" << endl;
  cout << " hardStop()" << endl;
  cout << " softHiZ()" << endl;
  cout << " hardHiZ()" << endl;
  cout << "------------------------------------------------------------" << endl << endl;

  cout << "============================================================" << endl;
}

// Really ugly, but hey it works and easy to understand what is going on
void handleMenuOption(const std::string &input , AutoDriver &driver)
{

    if      ( input.find("busyCheck")!= std::string::npos)
    {
	cout << "BusyState = " << driver.isBusy() << endl;
    }
    else if ( input.find("getStatus")!= std::string::npos)
    {
        cout << "Status = " << driver.getStatus() << endl;
    }
    else if ( input.find("setParam")!= std::string::npos )
    {
        std::vector<long> arguments = getArguments(input,2);
	driver.setParam(static_cast<ParamRegister> (arguments[0]),arguments[1]);
    }
    else if ( input.find("setLoSpdOpt")!= std::string::npos )
    {
        assert(!"TODO");
    }
    else if ( input.find("configSyncPin")!= std::string::npos )
    {
        assert(!"TODO");
    }
    else if ( input.find("configStepMode")!= std::string::npos )
    {
        std::vector<long> arguments = getArguments(input,1);
	driver.configStepMode(static_cast<StepMode>(arguments[0]));
    }
    else if ( input.find("setMaxSpeed")!= std::string::npos )
    {
        std::vector<long> arguments = getArguments(input,1);
        driver.setMaxSpeed((uint8_t)arguments[0]);
    }
    else if ( input.find("setMinSpeed")!= std::string::npos )
    {
        std::vector<long> arguments = getArguments(input,1);
        driver.setMinSpeed((uint8_t)arguments[0]);
    }
    else if ( input.find("setFullSpeed")!= std::string::npos )
    {
        std::vector<long> arguments = getArguments(input,1);
        driver.setFullSpeed((uint8_t)arguments[0]);
    }
    else if ( input.find("setAcc")!= std::string::npos )
    {
        std::vector<long> arguments = getArguments(input,1);
        driver.setAcc((uint8_t)arguments[0]);
    }
    else if ( input.find("setDec")!= std::string::npos )
    {
        std::vector<long> arguments = getArguments(input,1);
        driver.setDec((uint8_t)arguments[0]);
    }
    else if ( input.find("setOCThreshold")!= std::string::npos )
    {
        assert (!"TODO");
    }
    else if ( input.find( "setPWMFreq")!= std::string::npos )
    {
        assert (!"TODO");
    }
    else if ( input.find("setSlewRate")!= std::string::npos )
    {
        assert (!"TODO");
    }
    else if ( input.find("setOCShutdown")!= std::string::npos )
    {
        assert (!"TODO");
    }
    else if ( input.find("setVoltageComp")!= std::string::npos )
    {
        assert (!"TODO");
    }
    else if ( input.find("setSwitchMode")!= std::string::npos )
    {
        assert (!"TODO");
    }
    else if ( input.find("setOscMode")!= std::string::npos )
    {
        assert (!"TODO");
    }
    else if ( input.find("setAccKVAL")!= std::string::npos )
    {
        assert (!"TODO");
    }
    else if ( input.find("setDecKVAL")!= std::string::npos )
    {
        assert (!"TODO");
    }
    else if ( input.find("setRunKVAL")!= std::string::npos )
    {
        assert (!"TODO");
    }
    else if ( input.find("setHoldKVAL")!= std::string::npos )
    {
        assert (!"TODO");
    }
    else if ( input.find("getParam")!= std::string::npos )
    {
        std::vector<long> arguments = getArguments(input,1);
	cout << "Param " << arguments[0] << " = " << driver.getParam(static_cast<ParamRegister>(arguments[0])) << endl;
    }
    else if ( input.find("getLoSpdOpt")!= std::string::npos )
    {
        assert (!"TODO");
    }
    else if ( input.find("getStepMode")!= std::string::npos )
    {
        cout << "Step Mode = " << driver.getStepMode() << endl;
    }
    else if ( input.find("getMaxSpeed")!= std::string::npos )
    {
        cout << "Max Speed = " << driver.getMaxSpeed() << endl;
    }
    else if ( input.find("getMinSpeed")!= std::string::npos )
    {
        std::vector<long> arguments = getArguments(input,1);
        cout << "Min Speed = " << driver.getMinSpeed() << endl;
    }
    else if ( input.find("getFullSpeed")!= std::string::npos )
    {
        cout << "Full Speed = " << driver.getFullSpeed() << endl;
    }
    else if ( input.find("getAcc")!= std::string::npos )
    {
        cout << "Acc = " << driver.getAcc() << endl;
    }
    else if ( input.find("getDec")!= std::string::npos )
    {
        cout << "Dec = " << driver.getDec() << endl;
    }
    else if ( input.find("getOCThreshold(!= std::string::npos )")!= std::string::npos )
    {
        assert (!"TODO");
    }
    else if ( input.find("getPWMFreqDivisor(!= std::string::npos )")!= std::string::npos )
    {
        assert (!"TODO");
    }
    else if ( input.find("getPWMFreqMultiplier(!= std::string::npos )")!= std::string::npos )
    {
        assert (!"TODO");
    }
    else if ( input.find("getSlewRate(!= std::string::npos )")!= std::string::npos )
    {
        assert (!"TODO");
    }
    else if ( input.find("getOCShutdown(!= std::string::npos )")!= std::string::npos )
    {
        assert (!"TODO");
    }
    else if ( input.find("getVoltageComp(!= std::string::npos )")!= std::string::npos )
    {
        assert (!"TODO");
    }
    else if ( input.find("getSwitchMode(!= std::string::npos )")!= std::string::npos )
    {
        assert (!"TODO");
    }
    else if ( input.find("getOscMode(!= std::string::npos )")!= std::string::npos )
    {
        assert (!"TODO");
    }
    else if ( input.find("getAccKVAL(!= std::string::npos )")!= std::string::npos )
    {
        assert (!"TODO");
    }
    else if ( input.find("getDecKVAL(!= std::string::npos )")!= std::string::npos )
    {
        assert (!"TODO");
    }
    else if ( input.find("getRunKVAL(!= std::string::npos )")!= std::string::npos )
    {
        assert (!"TODO");
    }
    else if ( input.find("getHoldKVAL(!= std::string::npos )")!= std::string::npos )
    {
        assert (!"TODO");
    }
    else if ( input.find("getPos")!= std::string::npos )
    {
        cout << "Pos = " << driver.getPos() << endl;
    }
    else if ( input.find("getMark")!= std::string::npos )
    {
        cout << "Mark = " << driver.getMark() << endl;
    }
    else if ( input.find("run")!= std::string::npos )
    {
        std::vector<long> arguments = getArguments(input,2);
	driver.run(static_cast<MotorSpinDirection>(arguments[0]),arguments[1]);
    }
    else if ( input.find("stepClock")!= std::string::npos )
    {
        std::vector<long> arguments = getArguments(input,1);
	driver.stepClock(static_cast<MotorSpinDirection>(arguments[0]));
    }
    else if ( input.find("move")!= std::string::npos )
    {
        std::vector<long> arguments = getArguments(input,2);
	driver.move(static_cast<MotorSpinDirection>(arguments[0]),arguments[1]);
    }
    else if ( input.find("goTo")!= std::string::npos )
    {
        std::vector<long> arguments = getArguments(input,1);
        driver.goTo(arguments[0]);
    }
    else if ( input.find("goToDir")!= std::string::npos )
    {
        std::vector<long> arguments = getArguments(input,2);
	driver.goToDir(static_cast<MotorSpinDirection>(arguments[0]),arguments[1]);
    }
    else if ( input.find("goUntil")!= std::string::npos )
    {
        std::vector<long> arguments = getArguments(input,3);
	driver.goUntil(static_cast<MotorSpinDirection>(arguments[0]),arguments[1],static_cast<Action>(arguments[2]));
    }
    else if ( input.find("releaseSw")!= std::string::npos )
    {
        std::vector<long> arguments = getArguments(input,2);
	driver.releaseSw(static_cast<MotorSpinDirection>(arguments[0]),static_cast<Action>(arguments[1]));
    }
    else if ( input.find("goHome")!= std::string::npos )
    {
        driver.goHome();
    }
    else if ( input.find("goMark")!= std::string::npos )
    {
        driver.goMark();
    }
    else if ( input.find("setMark")!= std::string::npos )
    {
        std::vector<long> arguments = getArguments(input,1);
        driver.setMark(arguments[0]);
    }
    else if ( input.find("setPos")!= std::string::npos )
    {
        std::vector<long> arguments = getArguments(input,1);
        driver.setPos(arguments[0]);
    }
    else if ( input.find("resetPos")!= std::string::npos )
    {
        driver.resetPos();
    }
    else if ( input.find("resetDev")!= std::string::npos )
    {
        driver.resetDev();
    }
    else if ( input.find("soft")!= std::string::npos )
    {
        driver.softStop();
    }
    else if ( input.find("hardStop")!= std::string::npos )
    {
        driver.hardStop();
    }
    else if ( input.find("softHiZ")!= std::string::npos )
    {
        driver.softHiZ();
    }
    else if ( input.find("hardHiZ")!= std::string::npos )
    {
        driver.hardHiZ();
    }
    else
    {
        throw std::runtime_error("Unable to parse input command " + input);
    }

}

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
