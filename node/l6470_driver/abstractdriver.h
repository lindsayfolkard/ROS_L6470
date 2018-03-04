#pragma once

#include "motor.h"
#include "types.h"
#include "config.h"
#include "commsdriver.h" // commsdebug level (probs put in types.h)

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

    virtual ~AbstractDriver(){}

    /////////////////////////
    /// Status Commands
    /////////////////////////

    // Contains all data in status command
    // and also the current position and speed
    virtual std::vector<Status> getStatus() = 0;
    virtual Status getStatus(int motor) = 0;

    // Get status from every motor and clear the status flags...
    virtual std::vector<Status> clearStatus() = 0;

    // Individual get functions if only very specific data needed
    virtual std::vector<bool> isBusy() = 0;
    virtual bool isBusy(int motor) = 0;

    virtual std::vector<int32_t> getPos() = 0;   // steps
    virtual int32_t getPos(int motor) = 0;

    virtual std::vector<uint32_t> getSpeed() = 0; // steps/s
    virtual uint32_t getSpeed(int motor) = 0;

    virtual std::vector<int32_t> getMark() = 0;  // steps?
    virtual int32_t getMark(int motor) = 0;

    ////////////////////////////////////////
    /// Profile Configuration Commands
    ////////////////////////////////////////

    virtual void setConfig(AbstractConfig &config , int motor) = 0;

    // Profile is different, this we want to set efficiently since it is real-time critical
    virtual void setProfileCfg(const std::map<int,ProfileCfg> &cfgs) = 0;
    virtual void setProfileCfg(const ProfileCfg &cfg , int motor) = 0;

    //Profile Raw parameters
    virtual void setAcc(std::map<int,float> &accelerations) = 0;
    virtual void setAcc(float stepsPerSecondPerSecond , int motor ) = 0;

    virtual void setDec(std::map<int,float> &decelerations) = 0;
    virtual void setDec(float stepsPerSecondPerSecond , int motor ) = 0;

    virtual void setMaxSpeed(const std::map <int,float> &maxSpeeds) = 0;
    virtual void setMaxSpeed(float stepsPerSecond , int motor) = 0;

    virtual void setMinSpeed(const std::map <int,float> &minSpeeds) = 0;
    virtual void setMinSpeed(float stepsPerSecond , int motor) = 0;

    virtual ProfileCfg getProfileCfg(int motor) = 0;

    // void setFullSpeed(float stepsPerSecond) = 0;

    //////////////////////////////
    /// Operational Commands
    /////////////////////////////

    // Speed Commands
    virtual void run(const std::map<int, DataCommand> &runCommands) = 0;
    virtual void run(const RunCommand &runCommand , int motor) = 0;

    virtual void goUntil(const std::map<int, DataCommand> &goUntilCommands) = 0;
    virtual void goUntil(const GoUntilCommand &command , int motor) = 0;

//    void releaseSw(const std::map <int,ReleaseSwCommand> &releaseSWCommands) = 0;
//    void releaseSw(const ReleaseSwCommand &command , int motor) = 0;

    // Position Commands
    virtual void move(const std::map <int,DataCommand> &moveCommands) = 0;
    virtual void move(const MoveCommand &command , int motor) = 0;

    virtual void goTo(const std::map <int,DataCommand> &goToCommands) = 0;
    virtual void goTo(const GoToCommand &command , int motor) = 0;

    virtual void goToDir(const std::map <int,DataCommand> &goToDirCommands) = 0;
    virtual void goToDir(const GoToDirCommand &command , int motor) = 0;

    virtual void goHome(const std::vector <int> &motors) = 0;
    virtual void goHome(int motor) = 0;

    virtual void goMark(const std::vector <int> &motors) = 0;
    virtual void goMark(int motor) = 0;

    // Stop Commands
    virtual void stopAllHard() = 0;
    virtual void stopAllSoft() = 0;

    virtual void softStop(const std::vector <int> &motors) = 0;
    virtual void softStop(int motor) = 0;

    virtual void hardStop(const std::vector <int> &motors) = 0;
    virtual void hardStop(int motor) = 0;

    virtual void softHiZ(const std::vector <int> &motors) = 0;
    virtual void softHiZ(int motor) = 0;

    virtual void hardHiZ(const std::vector <int> &motors) = 0;
    virtual void hardHiZ(int motor) = 0;

    // Set Commands
    //void setMark(const std::map<int, long> &marks) = 0;
    //void setPos(const std::map<int,long> &newPositions) = 0;
    virtual void setPos  (int32_t pos , int motor) = 0;
    virtual void setAllPos(int32_t pos) = 0;
    virtual void resetPos(const std::vector <int> &motors) = 0;
    virtual void resetPos(int motor) = 0;
    virtual void resetDev(const std::vector <int> &motors) = 0;

};
