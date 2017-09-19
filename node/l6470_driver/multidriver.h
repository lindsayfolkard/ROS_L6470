#pragma once

#include <mraa.hpp>
#include "types.h"
#include "motor.h"
#include "support.h"
#include "commands.h"
#include <memory>
#include <stdint.h>
#include <vector>
#include <map>
#include <assert.h>

// MultiDriver
// Description : A single class which manages all daisy chained motors at once
// Rationale   : The L6470 uses a daisy chaining approach to handle comms. This daisy chaining is a bit tricky
//               and is hard to separate into parts without giving redundant comms.
// FML - this is a real bitch...!!

class MultiDriver
{
  public:

    /////////////////////////
    /// Constructors
    /////////////////////////

    MultiDriver(const std::vector<StepperMotor> &motors, int chipSelectPin, int resetPin, int busyPin = -1 ,CommsDebugLevel commsDebugLevel = CommsDebugNothing);

    MultiDriver(const std::vector<StepperMotor> &motors,
                const std::vector<Config> &configs,
                int chipSelectPin,
                int resetPin,
                int busyPin = -1,
                CommsDebugLevel commsDebugLevel = CommsDebugNothing);
    
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

    /////////////////////////
    /// Configuration Commands
    /////////////////////////

    // Config is a once off thing, no need to make it efficient.
    // and im lazy, so fuck it
    void setConfig(const Config &cfg , int motor);
    Config getConfig(int motor);

    void setBackEmfConfig(const VoltageModeCfg &backEmfConfig , int motor);
    VoltageModeCfg getBackEmfConfig(int motor);

    ///
    /// Profile Configuration Commands
    ///

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
    
    /////////////////////////
    /// Operational Commands
    ////////////////////////

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
    void setPos(int32_t pos , int motor);
    void resetPos(const std::vector <int> &motors);
    void resetPos(int motor);
    void resetDev(const std::vector <int> &motors);

    /////////////////////////
    /// Raw Access Set/Get Commands
    ////////////////////////
    void setParam(ParamRegister param, std::map<int, uint32_t> &values);
    void setParam(ParamRegister param , uint32_t value , int motor);
    std::vector<uint32_t> getParam(ParamRegister param);
    uint32_t getParam(ParamRegister param , int motor);
    
    void setLoSpdOpt(bool enable , int motor );
    void setSyncSelect( SyncSelect syncSelect , bool syncEnable , int motor );
    void setStepMode(StepMode stepMode , int motor );
    void setFullSpeed(float stepsPerSecond , int motor );

    void setOCThreshold(CurrentThreshold ocThreshold , int motor );
    void setStallThreshold(CurrentThreshold stallCurrent , int motor );
    void setPWMFreq(PwmFrequencyDivider divider, PwmFrequencyMultiplier multiplier , int motor );
    void setSlewRate(SlewRate slewRate , int motor );
    void setOCShutdown(OverCurrentDetection overCurrentDetection , int motor );
    void setVoltageComp(VoltageCompensation vsCompMode , int motor );
    void setSwitchMode(SwitchConfiguration switchMode , int motor );
    void setOscMode(OscillatorSelect oscillatorMode , int motor );
    void setAlarmState(AlarmState alarmState , int motor );

    void setAccKVAL(uint8_t kvalInput , int motor );
    void setDecKVAL(uint8_t kvalInput , int motor );
    void setRunKVAL(uint8_t kvalInput , int motor );
    void setHoldKVAL(uint8_t kvalInput , int motor );

    bool  getLoSpdOpt( int motor );
    float getMaxSpeed( int motor );
    float getMinSpeed( int motor );
    float getFullSpeed( int motor );
    float getAcc( int motor );
    float getDec( int motor );

    StepMode		   getStepMode( int motor );
    SyncSelect		   getSyncSelect( int motor );
    bool                   getSyncEnable( int motor );
    CurrentThreshold	   getOCThreshold( int motor );
    CurrentThreshold	   getStallThreshold( int motor );
    PwmFrequencyDivider	   getPWMFreqDivisor( int motor );
    PwmFrequencyMultiplier getPWMFreqMultiplier( int motor );
    SlewRate		   getSlewRate( int motor );
    OverCurrentDetection   getOCShutdown( int motor );
    VoltageCompensation	   getVoltageComp( int motor );
    SwitchConfiguration	   getSwitchMode( int motor );
    OscillatorSelect	   getOscMode( int motor );
    AlarmState		   getAlarmState( int motor );

    uint8_t getAccKVAL( int motor );
    uint8_t getDecKVAL( int motor );
    uint8_t getRunKVAL( int motor );
    uint8_t getHoldKVAL( int motor );
        
  private:

    // Sends requestValue to all motors
    std::vector<uint32_t> SPIXfer(uint8_t requestValue);

    // Send requestValue to only the specified motor
    uint32_t SPIXfer(uint8_t requestValue, int motor);

    std::vector<uint32_t> SPIXfer(const std::map<int, uint32_t> &data , int bitLength);
    std::vector<uint32_t> xferParam(const std::map<int,uint32_t> &parameters, uint8_t bitLen);

    void sendCommands(const std::map <int,DataCommand> &dataCommands);
    void sendCommands(const std::vector<int> &motors , uint8_t commandByte);
    void sendCommand(const DataCommand &dataCommand , int motor);

    void checkMotorIsValid(int motor);

    // Template validity check
    template <typename T> void checkMapIsValid (const std::map <int,T> &input)
    {
        if (input.size() > motors_.size())
        {
            assert (!"Invalid map size");
        }

        for (const auto &element : input)
        {
            if ((unsigned int) element.first >= motors_.size() || element.first < 0)
            {
                assert(!"Invalid map entry");
            }
        }
    }

    const std::vector<StepperMotor> motors_;
    int chipSelectPin_;
    int resetPin_;
    int busyPin_;
    CommsDebugLevel commsDebugLevel_;
    std::unique_ptr<mraa::Spi> SPI_;

};

/// #TODO - make pin mapping dynamic as we cannot know in advance how someone would use it in reality.
// configSyncPin() options: the !BUSY/SYNC pin can be configured to be low when
//  the chip is executing a command, *or* to output a pulse on each full step
//  clock (with some divisor). These 
#define BUSY_PIN   0x00     // !BUSY/SYNC pin set to !BUSY mode
#define SYNC_PIN   0x80     // pin set to SYNC mode
