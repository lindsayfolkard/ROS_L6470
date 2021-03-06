#pragma once

#include <mraa.hpp>
#include "constants.h"
#include "motor.h"
#include "support.h"
#include <memory>
#include <stdint.h>

enum Action
{
    Action_Reset_AbsPos = 0x00,
    Action_Copy_AbsPos  = 0x08
};

class AutoDriver
{
  public:

    /////////////////////////
    /// Constructors
    /////////////////////////

    AutoDriver(int position, int CSPin, int resetPin, int busyPin = -1);
    
    /////////////////////////
    /// Status Commands
    /////////////////////////

    // Contains all data in status command
    // and also the current position and speed
    Status getStatus();

    // Individual get functions if only very specific data needed
    bool   isBusy();
    long   getPos(); // steps
    long   getSpeed(); // steps/s
    long   getMark(); // steps?

    /////////////////////////
    /// Configuration Commands
    /////////////////////////

    void   setConfig(const Config &cfg);
    Config getConfig();

    void       setProfileCfg(const ProfileCfg &cfg);
    ProfileCfg getProfileCfg();
    
    /////////////////////////
    /// Operational Commands
    ////////////////////////

    // Speed Commands
    void run(MotorSpinDirection direction, float stepsPerSec);
    void goUntil(MotorSpinDirection direction, float stepsPerSec, Action action);
    void stepClock(MotorSpinDirection direction);
    void releaseSw(MotorSpinDirection direction, Action action);

    // Position Commands
    void move(MotorSpinDirection direction, unsigned long numSteps);
    void goTo(long pos);
    void goToDir(MotorSpinDirection direction, long pos);
    void goHome();
    void goMark();

    // Set Commands
    void setMaxSpeed(float stepsPerSecond);
    void setMinSpeed(float stepsPerSecond);
    void setMark(long newMark);
    void setPos(long newPos);
    void resetPos();
    void resetDev();

    // Stop Commands
    void softStop();
    void hardStop();
    void softHiZ();
    void hardHiZ();

    /////////////////////////
    /// Raw Access Set/Get Commands
    ////////////////////////
    void setParam(ParamRegister param, unsigned long value);
    long getParam(ParamRegister param);
    
    void setLoSpdOpt(bool enable);
    void setSyncSelect( SyncSelect syncSelect , bool syncEnable);
    void setStepMode(StepMode stepMode);
    void setFullSpeed(float stepsPerSecond);
    void setAcc(float stepsPerSecondPerSecond);
    void setDec(float stepsPerSecondPerSecond);

    void setOCThreshold(CurrentThreshold ocThreshold);
    void setStallThreshold(CurrentThreshold stallCurrent);
    void setPWMFreq(PwmFrequencyDivider divider, PwmFrequencyMultiplier multiplier);
    void setSlewRate(SlewRate slewRate);
    void setOCShutdown(OverCurrentDetection overCurrentDetection);
    void setVoltageComp(VoltageCompensation vsCompMode);
    void setSwitchMode(SwitchConfiguration switchMode);
    void setOscMode(OscillatorSelect oscillatorMode);
    void setAlarmState(AlarmState alarmState);

    void setAccKVAL(uint8_t kvalInput);
    void setDecKVAL(uint8_t kvalInput);
    void setRunKVAL(uint8_t kvalInput);
    void setHoldKVAL(uint8_t kvalInput);
    void setBackEmfConfig(const BackEmfConfig &backEmfConfig);

    bool  getLoSpdOpt();
    float getMaxSpeed();
    float getMinSpeed();
    float getFullSpeed();
    float getAcc();
    float getDec();

    BackEmfConfig	   getBackEmfConfig();
    StepMode		   getStepMode();
    SyncSelect		   getSyncSelect();
    bool                   getSyncEnable();
    CurrentThreshold	   getOCThreshold();
    CurrentThreshold	   getStallThreshold();
    PwmFrequencyDivider	   getPWMFreqDivisor();
    PwmFrequencyMultiplier getPWMFreqMultiplier();
    SlewRate		   getSlewRate();
    OverCurrentDetection   getOCShutdown();
    VoltageCompensation	   getVoltageComp();
    SwitchConfiguration	   getSwitchMode();
    OscillatorSelect	   getOscMode();
    AlarmState		   getAlarmState();

    uint8_t getAccKVAL();
    uint8_t getDecKVAL();
    uint8_t getRunKVAL();
    uint8_t getHoldKVAL();
        
  private:

    uint8_t SPIXfer(uint8_t data);
    long xferParam(unsigned long value, uint8_t bitLen);
    long paramHandler(uint8_t param, unsigned long value);
     
    int _CSPin;
    int _resetPin;
    int _busyPin;
    int _position;
    static int _numBoards; // count of the number of boards instantiated --> remove (do we need this here)
    std::unique_ptr<mraa::Spi> _SPI;
};

/// #TODO - make pin mapping dynamic as we cannot know in advance how someone would use it in reality.
// configSyncPin() options: the !BUSY/SYNC pin can be configured to be low when
//  the chip is executing a command, *or* to output a pulse on each full step
//  clock (with some divisor). These 
#define BUSY_PIN   0x00     // !BUSY/SYNC pin set to !BUSY mode
#define SYNC_PIN   0x80     // pin set to SYNC mode
