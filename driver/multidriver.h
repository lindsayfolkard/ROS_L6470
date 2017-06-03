#pragma once

#include <mraa.hpp>
#include "constants.h"
#include "motor.h"
#include "support.h"
#include <memory>
#include <stdint.h>
#include <vector>
#include <map>

// Template validity check
template <typename T> void checkMapIsValid (const std::map <int,T> &input , int motorLength)
{
    if (input.size() > motorLength)
    {
        assert (!"Invalid map size");
    }

    for (auto pair : input)
    {
        if (input.first >= motorLength || input.first < 0)
        {
            assert(!"Invalid map entry");
        }
    }
}

// MultiDriver
// Description : A single class which manages all daisy chained motors at once
// Rationale   : The L6470 uses a daisy chaining approach to handle comms. This daisy chaining is a bit tricky
//               and is hard to separate into parts without giving redundant comms.

class MultiDriver
{
  public:

    /////////////////////////
    /// Constructors
    /////////////////////////

    MultiDriver(const std::vector<StepperMotor> &motors, int chipSelectPin, int resetPin, int busyPin = -1);

    MultiDriver(const std::vector<StepperMotor> &motors,
                const std::map<int, Config> &config,
                int chipSelectPin,
                int resetPin,
                int busyPin = -1);
    
    /////////////////////////
    /// Status Commands
    /////////////////////////

    // Contains all data in status command
    // and also the current position and speed
    std::vector<Status> getStatus();

    // Individual get functions if only very specific data needed
    std::vector<bool> isBusy();
    std::vector<long> getPos();   // steps
    std::vector<long> getSpeed(); // steps/s
    std::vector<long> getMark();  // steps?

    /////////////////////////
    /// Configuration Commands
    /////////////////////////

    void setConfig(const std::map<int,Config> &cfg);
    std::vector<Config>     getConfig();

    void setProfileCfg(const std::map<int,ProfileCfg> &cfg);
    std::vector<ProfileCfg> getProfileCfg();

    //    void setFullSpeed(float stepsPerSecond);
    //    void setAcc(float stepsPerSecondPerSecond);
    //    void setDec(float stepsPerSecondPerSecond);
    
    /////////////////////////
    /// Operational Commands
    ////////////////////////

    // Speed Commands
    void run(const std::map<int,RunCommand> &runCommands);
    void goUntil(const std::map <int,GoUntilCommand> &goUntilCommands);
    void stepClock(const std::map <int,MotorSpinDirection> &directions);
    void releaseSw(const std::map <int,GoUntilCommand> &releaseSWCommands);

    // Position Commands
    void move(const std::map <int,MoveCommand> &moveCommands);
    void goTo(const std::map <int,long> &positions);
    void goToDir(const std::map <int,GoToCommand> &goToCommands);
    void goHome(const std::map <int,bool> &goHome);
    void goMark(const std::map <int,bool> &goMark);

    // Set Commands
    void setMaxSpeed(const std::map <int,float> &maxSpeeds);
    void setMinSpeed(const std::map <int,float> &minSpeeds);
    void setMark(const std::map <int,long> &marks);
    void setPos(const std::map<int,long> &newPositions);
    void resetPos(const std::map<int,bool> &resetPosition);
    void resetDev(const std::map<int,bool> &resetDev);

    // Stop Commands
    void softStop(const std::map<int,bool> &resetPosition);
    void hardStop(const std::map<int,bool> &resetPosition);
    void softHiZ(const std::map<int,bool> &resetPosition);
    void hardHiZ(const std::map<int,bool> &resetPosition);

    /////////////////////////
    /// Raw Access Set/Get Commands
    ////////////////////////
    void setParam(ParamRegister param, std::map<int, T> &values);
    std::vector<T> getParam(ParamRegister param);
    
    //    void setLoSpdOpt(bool enable);
    //    void setSyncSelect( SyncSelect syncSelect , bool syncEnable);
    //    void setStepMode(StepMode stepMode);
    //    void setFullSpeed(float stepsPerSecond);
    //    void setAcc(float stepsPerSecondPerSecond);
    //    void setDec(float stepsPerSecondPerSecond);

    //    void setOCThreshold(CurrentThreshold ocThreshold);
    //    void setStallThreshold(CurrentThreshold stallCurrent);
    //    void setPWMFreq(PwmFrequencyDivider divider, PwmFrequencyMultiplier multiplier);
    //    void setSlewRate(SlewRate slewRate);
    //    void setOCShutdown(OverCurrentDetection overCurrentDetection);
    //    void setVoltageComp(VoltageCompensation vsCompMode);
    //    void setSwitchMode(SwitchConfiguration switchMode);
    //    void setOscMode(OscillatorSelect oscillatorMode);
    //    void setAlarmState(AlarmState alarmState);

    //    void setAccKVAL(uint8_t kvalInput);
    //    void setDecKVAL(uint8_t kvalInput);
    //    void setRunKVAL(uint8_t kvalInput);
    //    void setHoldKVAL(uint8_t kvalInput);
    //    void setBackEmfConfig(const BackEmfConfig &backEmfConfig);

    //    bool  getLoSpdOpt();
    //    float getMaxSpeed();
    //    float getMinSpeed();
    //    float getFullSpeed();
    //    float getAcc();
    //    float getDec();

    //    BackEmfConfig	   getBackEmfConfig();
    //    StepMode		   getStepMode();
    //    SyncSelect		   getSyncSelect();
    //    bool                   getSyncEnable();
    //    CurrentThreshold	   getOCThreshold();
    //    CurrentThreshold	   getStallThreshold();
    //    PwmFrequencyDivider	   getPWMFreqDivisor();
    //    PwmFrequencyMultiplier getPWMFreqMultiplier();
    //    SlewRate		   getSlewRate();
    //    OverCurrentDetection   getOCShutdown();
    //    VoltageCompensation	   getVoltageComp();
    //    SwitchConfiguration	   getSwitchMode();
    //    OscillatorSelect	   getOscMode();
    //    AlarmState		   getAlarmState();

    //    uint8_t getAccKVAL();
    //    uint8_t getDecKVAL();
    //    uint8_t getRunKVAL();
    //    uint8_t getHoldKVAL();
        
  private:

    std::vector<uint8_t> SPIXfer(const std::map<int,uint8_t> &data);
    std::vector<long> xferParam(const std::map<int,unsigned long> &parameters, uint8_t bitLen);
    std::vector<long> paramHandler(uint8_t param, unsigned long value);
     
    int chipSelectPin_;
    int resetPin_;
    int busyPin_;
    std::unique_ptr<mraa::Spi> SPI_;
    const std::vector<StepperMotor> motors_;
};

/// #TODO - make pin mapping dynamic as we cannot know in advance how someone would use it in reality.
// configSyncPin() options: the !BUSY/SYNC pin can be configured to be low when
//  the chip is executing a command, *or* to output a pulse on each full step
//  clock (with some divisor). These 
#define BUSY_PIN   0x00     // !BUSY/SYNC pin set to !BUSY mode
#define SYNC_PIN   0x80     // pin set to SYNC mode
