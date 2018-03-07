#pragma once
#include "abstractdriver.h"

///
/// \brief The simdriver class
/// \abstract A simulator implementation of the abstract driver for use with robot in simulation mode
class SimDriver : public AbstractDriver
{
public:

    SimDriver(const std::vector<StepperMotor> &motors);

    // Contains all data in status command
    // and also the current position and speed
    virtual std::vector<Status> getStatus() override ;
    virtual Status getStatus(int motor) override ;

    virtual std::vector<Status> clearStatus() override;

    // Individual get functions if only very specific data needed
    virtual std::vector<bool> isBusy() override ;
    virtual bool isBusy(int motor) override ;

    virtual std::vector<int32_t> getPos() override ;   // steps
    virtual int32_t getPos(int motor) override ;

    virtual std::vector<uint32_t> getSpeed() override ; // steps/s
    virtual uint32_t getSpeed(int motor) override ;

    virtual std::vector<int32_t> getMark() override ;  // steps?
    virtual int32_t getMark(int motor) override ;

    ////////////////////////////////////////
    /// Profile Configuration Commands
    ////////////////////////////////////////

    virtual void setConfig(AbstractConfig &config , int motor) override ;

    // Profile is different, this we want to set efficiently since it is real-time critical
    virtual void setProfileCfg(const std::map<int,ProfileCfg> &cfgs) override ;
    virtual void setProfileCfg(const ProfileCfg &cfg , int motor) override ;

    //Profile Raw parameters
    virtual void setAcc(std::map<int,float> &accelerations) override ;
    virtual void setAcc(float stepsPerSecondPerSecond , int motor ) override ;

    virtual void setDec(std::map<int,float> &decelerations) override ;
    virtual void setDec(float stepsPerSecondPerSecond , int motor ) override ;

    virtual void setMaxSpeed(const std::map <int,float> &maxSpeeds) override ;
    virtual void setMaxSpeed(float stepsPerSecond , int motor) override ;

    virtual void setMinSpeed(const std::map <int,float> &minSpeeds) override ;
    virtual void setMinSpeed(float stepsPerSecond , int motor) override ;

    virtual ProfileCfg getProfileCfg(int motor) override ;

    // void setFullSpeed(float stepsPerSecond) override ;

    //////////////////////////////
    /// Operational Commands
    /////////////////////////////

    // Speed Commands
    virtual void run(const std::map<int, RunCommand> &runCommands) override ;
    virtual void run(const RunCommand &runCommand , int motor) override ;

    virtual void goUntil(const std::map<int, GoUntilCommand> &goUntilCommands) override ;
    virtual void goUntil(const GoUntilCommand &command , int motor) override ;

//    void releaseSw(const std::map <int,ReleaseSwCommand> &releaseSWCommands) override ;
//    void releaseSw(const ReleaseSwCommand &command , int motor) override ;

    // Position Commands
    virtual void move(const std::map <int,MoveCommand> &moveCommands) override ;
    virtual void move(const MoveCommand &command , int motor) override ;

    virtual void goTo(const std::map <int,GoToCommand> &goToCommands) override ;
    virtual void goTo(const GoToCommand &command , int motor) override ;

    virtual void goToDir(const std::map <int,GoToDirCommand> &goToDirCommands) override ;
    virtual void goToDir(const GoToDirCommand &command , int motor) override ;

    virtual void goHome(const std::vector <int> &motors) override ;
    virtual void goHome(int motor) override ;

    virtual void goMark(const std::vector <int> &motors) override ;
    virtual void goMark(int motor) override ;

    // Stop Commands
    virtual void stopAllHard() override;
    virtual void stopAllSoft() override;

    virtual void softStop(const std::vector <int> &motors) override ;
    virtual void softStop(int motor) override ;

    virtual void hardStop(const std::vector <int> &motors) override ;
    virtual void hardStop(int motor) override ;

    virtual void softHiZ(const std::vector <int> &motors) override ;
    virtual void softHiZ(int motor) override ;

    virtual void hardHiZ(const std::vector <int> &motors) override ;
    virtual void hardHiZ(int motor) override ;

    // Set Commands
    virtual void setMark(int32_t pos, int motor) override;
    virtual void setPos  (int32_t pos , int motor) override ;
    virtual void setAllPos(int32_t pos) override;
    virtual void resetPos(const std::vector <int> &motors) override ;
    virtual void resetPos(int motor) override ;
    virtual void resetDev(const std::vector <int> &motors) override ;

private:

    std::vector<StepperMotor> motors_;
};
