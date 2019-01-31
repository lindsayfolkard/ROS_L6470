#pragma once

#include "abstractdriver.h"
#include "types.h"
#include "motor.h"
#include "support.h"
#include "commands.h"
#include <memory>
#include <stdint.h>
#include <vector>
#include <map>
#include <assert.h>
#include "commsdriver.h"

// BaseDriver
// Description : A general implementation of the AbstractDriver
//               Encompasses all common commands.
class BaseDriver : public AbstractDriver
{

public:

    /////////////////////////
    /// Constructors
    /////////////////////////

    BaseDriver(const std::vector<StepperMotor> &motors, MotorDriverType motorDriverType, int spiBus = 0, CommsDebugLevel commsDebugLevel = CommsDebugNothing);

    /////////////////////////
    /// Status Commands
    /////////////////////////

    // Contains all data in status command
    // and also the current position and speed
    virtual std::vector<Status> getStatus() override;
    virtual Status getStatus(int motor) override;

    // Gets status and clears the status flags
    virtual std::vector<Status> clearStatus() override;

    // Individual get functions if only very specific data needed
    virtual std::vector<bool> isBusy() override;
    virtual bool isBusy(int motor) override;

    virtual std::vector<int32_t> getPos() override;   // steps
    virtual int32_t getPos(int motor) override;

    virtual std::vector<uint32_t> getSpeed() override; // steps/s
    virtual uint32_t getSpeed(int motor) override;

    virtual std::vector<int32_t> getMark() override;  // steps?
    virtual int32_t getMark(int motor) override;

    ////////////////////////////////////////
    /// Profile Configuration Commands
    ////////////////////////////////////////

    // Profile is different, this we want to set efficiently since it is real-time critical
    virtual void setProfileCfg(const std::map<int,ProfileCfg> &cfgs) override;
    virtual void setProfileCfg(const ProfileCfg &cfg , int motor) override;

    //Profile Raw parameters
    virtual void setAcc(std::map<int,float> &accelerations) override;
    virtual void setAcc(float stepsPerSecondPerSecond , int motor ) override;

    virtual void setDec(std::map<int,float> &decelerations) override;
    virtual void setDec(float stepsPerSecondPerSecond , int motor ) override;

    virtual void setMaxSpeed(const std::map <int,float> &maxSpeeds) override;
    virtual void setMaxSpeed(float stepsPerSecond , int motor) override;

    virtual void setMinSpeed(const std::map <int,float> &minSpeeds) override;
    virtual void setMinSpeed(float stepsPerSecond , int motor) override;

    float getMaxSpeed( int motor );
    float getMinSpeed( int motor );
    float getFullSpeed( int motor );
    float getAcc( int motor );
    float getDec( int motor );

    virtual ProfileCfg getProfileCfg(int motor) override;

    //////////////////////////////
    /// Operational Commands
    /////////////////////////////

    // Speed Commands
    virtual void run(const std::map<int,RunCommand> &runCommands) override;
    virtual void run(const RunCommand &runCommand , int motor) override;

    virtual void goUntil(const std::map<int,GoUntilCommand> &goUntilCommands) override;
    virtual void goUntil(const GoUntilCommand &command , int motor) override;

    //    void releaseSw(const std::map <int,ReleaseSwCommand> &releaseSWCommands) override;
    //    void releaseSw(const ReleaseSwCommand &command , int motor) override;

    // Position Commands
    virtual void move(const std::map <int,MoveCommand> &moveCommands) override;
    virtual void move(const MoveCommand &command , int motor) override;

    virtual void goTo(const std::map <int,GoToCommand> &goToCommands) override;
    virtual void goTo(const GoToCommand &command , int motor) override;

    virtual void goToDir(const std::map <int,GoToDirCommand> &goToDirCommands) override;
    virtual void goToDir(const GoToDirCommand &command , int motor) override;

    virtual void goHome(const std::vector <int> &motors) override;
    virtual void goHome(int motor) override;

    virtual void goMark(const std::vector <int> &motors) override;
    virtual void goMark(int motor) override;

    // Stop Commands
    virtual void stopAllHard() override;
    virtual void stopAllSoft() override;

    virtual void softStop(const std::vector <int> &motors) override;
    virtual void softStop(int motor) override;

    virtual void hardStop(const std::vector <int> &motors) override;
    virtual void hardStop(int motor) override;

    virtual void softHiZ(const std::vector <int> &motors) override;
    virtual void softHiZ(int motor) override;

    virtual void hardHiZ(const std::vector <int> &motors) override;
    virtual void hardHiZ(int motor) override;

    // Set Commands
    virtual void setMark (int32_t pos, int motor) override;
    virtual void setPos  (int32_t pos , int motor) override;
    virtual void setAllPos(int32_t pos) override;
    virtual void resetPos(const std::vector <int> &motors) override;
    virtual void resetPos(int motor) override;
    virtual void resetDev(const std::vector <int> &motors) override;

    // Stepper motors being controlled by this driver
    const std::vector<StepperMotor> motors_;
    std::unique_ptr<CommsDriver>    commsDriver_;

protected:

    void checkMotorIsValid(int motor);
    const MotorDriverType           motorDriverType_;
    CommsDebugLevel                 commsDebugLevel_;

};

class WrongMotorException : public std::exception
{
    public:

    WrongMotorException (const std::string &reason) : reason_(reason){}

    virtual const char * what() const throw() { return reason_.c_str();}

    private:
        const std::string reason_;
};

/// #TODO - make pin mapping dynamic as we cannot know in advance how someone would use it in reality.
// configSyncPin() options: the !BUSY/SYNC pin can be configured to be low when
//  the chip is executing a command, *or* to output a pulse on each full step
//  clock (with some divisor). These 
#define BUSY_PIN   0x00     // !BUSY/SYNC pin set to !BUSY mode
#define SYNC_PIN   0x80     // pin set to SYNC mode
