#pragma once

#include "types.h"

// AbstractDriver
// Description : The common interface used to control L647*, L648*, PowerStep01
// Rationale   : The STMicro stepper motors uses a daisy chaining approach to handle comms. This daisy chaining is a bit tricky
//               and is hard to separate into parts without giving redundant comms.

class AbstractDriver
{
  public:

    /////////////////////////
    /// Constructors
    /////////////////////////

    AbstractDriver(const std::vector<StepperMotor> &motors, int chipSelectPin, int resetPin, int busyPin = -1 ,CommsDebugLevel commsDebugLevel = CommsDebugNothing);

    /////////////////////////
    /// Status Commands
    /////////////////////////

    // Contains all data in status command
    // and also the current position and speed
    std::vector<Status> getStatus();
    Status getStatus(int motor);

    // Individual get functions if only very specific data needed
    std::vector<bool> isBusy();
    bool isBusy(int motor);

    std::vector<int32_t> getPos();   // steps
    int32_t getPos(int motor);

    std::vector<uint32_t> getSpeed(); // steps/s
    uint32_t getSpeed(int motor);

    std::vector<int32_t> getMark();  // steps?
    int32_t getMark(int motor);

    ////////////////////////////////////////
    /// Profile Configuration Commands
    ////////////////////////////////////////

    // Profile is different, this we want to set efficiently since it is real-time critical
    void setProfileCfg(const std::map<int,ProfileCfg> &cfgs);
    void setProfileCfg(const ProfileCfg &cfg , int motor);

    //Profile Raw parameters
    void setAcc(std::map<int,float> &accelerations);
    void setAcc(float stepsPerSecondPerSecond , int motor );

    void setDec(std::map<int,float> &decelerations);
    void setDec(float stepsPerSecondPerSecond , int motor );

    void setMaxSpeed(const std::map <int,float> &maxSpeeds);
    void setMaxSpeed(float stepsPerSecond , int motor);

    void setMinSpeed(const std::map <int,float> &minSpeeds);
    void setMinSpeed(float stepsPerSecond , int motor);

    ProfileCfg getProfileCfg(int motor);

    // void setFullSpeed(float stepsPerSecond);

    //////////////////////////////
    /// Operational Commands
    /////////////////////////////

    // Speed Commands
    void run(const std::map<int, DataCommand> &runCommands);
    void run(const RunCommand &runCommand , int motor);

    void goUntil(const std::map<int, DataCommand> &goUntilCommands);
    void goUntil(const GoUntilCommand &command , int motor);

//    void releaseSw(const std::map <int,ReleaseSwCommand> &releaseSWCommands);
//    void releaseSw(const ReleaseSwCommand &command , int motor);

    // Position Commands
    void move(const std::map <int,DataCommand> &moveCommands);
    void move(const MoveCommand &command , int motor);

    void goTo(const std::map <int,DataCommand> &goToCommands);
    void goTo(const GoToCommand &command , int motor);

    void goToDir(const std::map <int,DataCommand> &goToDirCommands);
    void goToDir(const GoToDirCommand &command , int motor);

    void goHome(const std::vector <int> &motors);
    void goHome(int motor);

    void goMark(const std::vector <int> &motors);
    void goMark(int motor);

    // Stop Commands
    void softStop(const std::vector <int> &motors);
    void softStop(int motor);

    void hardStop(const std::vector <int> &motors);
    void hardStop(int motor);

    void softHiZ(const std::vector <int> &motors);
    void softHiZ(int motor);

    void hardHiZ(const std::vector <int> &motors);
    void hardHiZ(int motor);

    // Set Commands
    //void setMark(const std::map<int, long> &marks);
    //void setPos(const std::map<int,long> &newPositions);
    void setPos  (int32_t pos , int motor);
    void resetPos(const std::vector <int> &motors);
    void resetPos(int motor);
    void resetDev(const std::vector <int> &motors);

    ///////////////////////////////////
    /// Raw Access Set/Get Commands
    ///////////////////////////////////
    void setParam(ParamRegister param, std::map<int, uint32_t> &values);
    void setParam(ParamRegister param , uint32_t value , int motor);
    std::vector<uint32_t> getParam(ParamRegister param);
    uint32_t getParam(ParamRegister param , int motor);

};


