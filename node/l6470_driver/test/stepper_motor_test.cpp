#include <iostream>
#include "../abstractdriver.h"
#include "../powerstepdriver.h"
#include "../motor.h"
#include <stdexcept>
#include <string>
#include <regex>
#include <assert.h>
#include <chrono>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/tokenizer.hpp>

using namespace std;

std::string getUsage(const std::string &funcName);
std::string printMenu();
void checkArguments(const std::vector<int> &arguments, int requiredLength);
bool hasOption(string input, string shortName, string longName);
std::vector<int> getArguments(const std::string &input);
void printState(PowerStepDriver &driver, const int motor, const int timeReadS);

int main (int argc, char ** argv)
{
    if (argc < 2)
    {
        std::cout << "Insufficient args. Require motor speed "  << std::endl;
        std::cout << "Usage " << argv[0] << " <configFile>" << std::endl;
        return EXIT_FAILURE;
    }

    // Parse the config for the controllers
    const std::string configFilePath = argv[1];
    OverallCfg overallCfg(configFilePath);

    std::vector<PowerStepCfg> powerStepCfgs;
    std::vector<StepperMotor> motors;
    for (const auto &file : overallCfg.cfgFiles_)
    {
        motors.push_back(StepperMotor(file));
        powerStepCfgs.push_back(PowerStepCfg(file));
    }

    // Instantiate the Power Step Driver
    cout << "Try to instantiate the driver" << endl;
    PowerStepDriver driver(motors,powerStepCfgs,overallCfg.spiBus_,overallCfg.commsDebugLevel_);
    driver.setAllPos(0);
    driver.softHiZ(0);
    cout << "Instantiated the driver!" << endl;
    cout << printMenu() << endl;

    // Parse Commands
    const int timeReadS = 5;
    while (true)
    {
        try
        {
            // Get user command
            std::string input;
            cout << ">>";
            cin  >> input;

            const std::vector<int> arguments = getArguments(input);

            if (hasOption(input,"gt","gotopos"))
            {
                cout << "GoToPos : " << input << endl;
                checkArguments(arguments,2);
                const int motor = arguments[0];
                const int pos   = arguments[1];
                driver.goTo(GoToCommand(pos),motor);
                printState(driver,motor,timeReadS); // Could just use a predicate. That way could stop when we reach the position...?
            }
            else if (hasOption(input,"gs","getstatus"))
            {
                cout << "GetStatus : " << input << endl;
                checkArguments(arguments,1);
                cout << "Status Motor " << arguments[0] << " : " << endl << driver.getStatus(arguments[0]) << endl;
            }
            else if (hasOption(input,"gp","getpos"))
            {
                cout << "GetPos : " << input << endl;
                checkArguments(arguments,1);
                cout << "Motor " << arguments[0] << " pos is : " << driver.getPos(arguments[0]) << endl;
            }
            else if (hasOption(input,"rs","runspeed"))
            {
                cout << "RunSpeed : " << input << endl;
                checkArguments(arguments,2);
                const int motor = arguments[0];
                const int speed = arguments[1];
                cout << "Let's run speed at " << speed << endl;
                driver.run(RunCommand(speed),motor);
                printState(driver,motor,timeReadS);
            }
            else if (hasOption(input,"sp","setprofile"))
            {
                cout << "SetProfile : " << input << endl;
                checkArguments(arguments,5);
                ProfileCfg profile;
                profile.acceleration = arguments[1];
                profile.deceleration = arguments[2];
                profile.maxSpeed     = arguments[3];
                profile.minSpeed     = arguments[4];
                driver.setProfileCfg(profile,arguments[0]);
            }
            else if (hasOption(input,"so","soft"))
            {
                driver.softHiZAll();
            }
            else if (hasOption(input,"s","stop"))
            {
                cout << "Stop : " << endl;
                driver.stopAllHard();
            }
            else
            {
                cout << " unrecognised command '" << input << "'" << endl;
                cout << printMenu() << endl;
            }

        }
        catch (std::exception &e)
        {
            cout << "Caught exception : " << e.what();
            driver.stopAllSoft();
        }
    }

    return EXIT_SUCCESS;
}

void checkArguments(const std::vector<int> &arguments, int requiredLength)
{
    if (arguments.size() != (size_t) requiredLength)
        throw std::runtime_error("Arguments size " + std::to_string(arguments.size()) + " does not equal the requires size of " + std::to_string(requiredLength));
}

bool hasOption(std::string input, std::string shortName, std::string longName)
{
    boost::algorithm::to_lower(input);
    boost::algorithm::to_lower(shortName);
    boost::algorithm::to_lower(longName);

    return ( input.find(shortName) != std::string::npos) ||
           ( input.find(longName)  != std::string::npos);
}

std::string printMenu()
{
    std::stringstream ss;
    ss << "gt or ('GoToPos')    ==> Motor,Position(deg)" << std::endl
       << "gs or ('GetState')   ==> optional(Motor)"     << std::endl
       << "gp or ('GetPos')     ==> Motor"               << std::endl
       << "rs or ('RunSpeed')   ==> Motor,Speed,optional(Time)"    << std::endl
       << "s  or ('Stop')       ==> optional(Motor)"     << std::endl
       << "so  or ('Soft')      "     << std::endl
       << "sp or ('SetProfile') ==> Motor,Accel,Decel,MaxSpeed,MinSpeed" << std::endl;

    return ss.str();
}

std::vector<int> getArguments(const std::string &input)
{
    boost::char_separator<char> sep(",");
    boost::tokenizer< boost::char_separator<char> > tok(input, sep);
    std::vector<int> tokens;

    int count=0;
    for (const std::string &token : tok)
    {
        if (count > 0)
            tokens.push_back(std::atoi(token.c_str()));

        ++count;
    }

    return tokens;
}

void printState(PowerStepDriver &driver, const int motor, const int timeReadS)
{
    const int sleepMicros = 250e3;
    int roughTimeCount=0;
    const int maxTimeCount = timeReadS*1e6;

    while(roughTimeCount < maxTimeCount)
    {
        cout << "Motor " << motor << " pos = " << driver.getPos(motor) << ", speed = " << driver.getSpeed(motor) << endl;
        usleep(sleepMicros);
        roughTimeCount+=sleepMicros;
    }
}

