#pragma once

#include "types.h"
#include "commsdriver.h"

// Abstract Classes for passing configs around
class AbstractConfig {
    virtual void set(CommsDriver &commsDriver, int motor) = 0;
    virtual void readFromFile(const std::string &filePath) = 0;
};

class WriteableConfig {

    virtual void writeToFile(const std::string &cfgFilePath) = 0;
};

enum MotorDriverType
{
    PowerStep01,
    L6470, // not checked as of yet
    L6472  // not checked as of yet
};
std::string toString(MotorDriverType motorDriverType);
inline std::ostream& operator<<(std::ostream& os, MotorDriverType x)
{
    return os << toString(x);
}
MotorDriverType motorDriverTypeFromString(const std::string &str);

struct CfgFile
{
    CfgFile(const std::string &stepperMotorFile,const std::string &customConfigFile, const std::string motorModel) :
        stepperMotorFile_(stepperMotorFile),
        customConfigFile_(customConfigFile),
        motorModel_(motorModel){}

    std::string stepperMotorFile_;
    std::string customConfigFile_;
    std::string motorModel_;
};

struct OverallCfg
{
    OverallCfg(const std::string &filePath);
    OverallCfg(const std::vector<CfgFile> &cfgFiles,
               MotorDriverType             controllerType,
               CommsDebugLevel             commsDebugLevel);

    void writeToFile(const std::string &baseFile);

    std::vector<CfgFile>     cfgFiles_;
    MotorDriverType          controllerType_;
    CommsDebugLevel          commsDebugLevel_;
};
std::string toString(const OverallCfg &cfg);
inline std::ostream& operator<<(std::ostream& os,const OverallCfg &x)
{
    return os << toString(x);
}

class CurrentModeCfg : public AbstractConfig,
                       public WriteableConfig
{

public:

    virtual void set(CommsDriver &commsDriver, int motor) override;
    virtual void readFromFile(const std::string &filePath) override;
    virtual void writeToFile(const std::string &filePath) override;

    uint8_t tvalHold;
    uint8_t tvalRun;
    uint8_t tvalAcc;
    uint8_t tvalDec;
    uint8_t tFast;
    uint8_t tonMin;
    uint8_t toffMin;

    // Config Register
    bool                  predictiveCurrentControlEnabled;
    TargetSwitchingPeriod targetSwitchingPeriod;
    SwitchConfiguration   switchConfiguration;
    bool                  enableTorqueRegulation;
    bool                  externalClockEnabled;

};

class VoltageModeCfg : public AbstractConfig,
                       public WriteableConfig
{
public:

    VoltageModeCfg();
    VoltageModeCfg(const std::string &file);
    
    virtual void set(CommsDriver &commsDriver, int motor) override;
    virtual void readFromFile(const std::string &file) override;
    virtual void writeToFile(const std::string &cfgFilePath) override;

    uint8_t holdingKVal;
    uint8_t constantSpeedKVal;
    uint8_t accelStartingKVal;
    uint8_t decelStartingKVal;

    uint32_t intersectSpeed;
    uint32_t startSlope;
    uint32_t accelFinalSlope;
    uint32_t decelFinalSlope;

    SlewRate               slewRate;
    VoltageCompensation    voltageCompensation;
    PwmFrequencyMultiplier pwmFrequencyMultiplier;
    PwmFrequencyDivider    pwmFrequencyDivider;
    bool                   enableLowSpeedOptimisation;

private:

    void setDefaults();
    
    // probs can just be external methods in their own namespace but fuck it
    void setAccKVAL(uint8_t kvalInput, CommsDriver &commsDriver, int motor );
    void setDecKVAL(uint8_t kvalInput, CommsDriver &commsDriver, int motor );
    void setRunKVAL(uint8_t kvalInput, CommsDriver &commsDriver, int motor );
    void setHoldKVAL(uint8_t kvalInput, CommsDriver &commsDriver, int motor );
    void setVoltageComp(VoltageCompensation vsCompMode, CommsDriver &commsDriver, int motor );
    void setPWMFreq(PwmFrequencyDivider divider, PwmFrequencyMultiplier multiplier,  CommsDriver &commsDriver, int motor );
    void setSlewRate(SlewRate slewRate, CommsDriver &commsDriver, int motor );

    uint8_t getAccKVAL(CommsDriver &commsDriver, int motor );
    uint8_t getDecKVAL(CommsDriver &commsDriver, int motor );
    uint8_t getRunKVAL(CommsDriver &commsDriver, int motor );
    uint8_t getHoldKVAL(CommsDriver &commsDriver, int motor );

    PwmFrequencyDivider	   getPWMFreqDivisor(CommsDriver &commsDriver, int motor );
    PwmFrequencyMultiplier getPWMFreqMultiplier(CommsDriver &commsDriver, int motor );
    VoltageCompensation	   getVoltageComp(CommsDriver &commsDriver, int motor );
    SlewRate            getSlewRate(CommsDriver &commsDriver, int motor );
};

std::string toString(const VoltageModeCfg &backEmfConfig);
inline std::ostream& operator<<(std::ostream& os,const VoltageModeCfg &x)
{
    return os << toString(x);
}

// General Static Config (meant to be set once at the start and then not really again)
// NB: valid parameters are always positive. A negative parameter is interpreted as do not set/read.
class CommonConfig : public AbstractConfig,
                     public WriteableConfig
{
public:

    CommonConfig(const std::string &file) { readFromFile(file); }
    CommonConfig(){ setDefaults(); }
    
    virtual void set(CommsDriver &commsDriver, int motor) override;
    virtual void readFromFile(const std::string &file) override;
    virtual void writeToFile(const std::string &cfgFilePath) override;

    int fullStepThresholdSpeed;
    int thermalDriftCoefficient;

    CurrentThreshold overCurrentThreshold;
    CurrentThreshold stallThreshold;

    // STEP_MODE register settings
    StepMode    stepMode;
    SyncSelect  syncSelect;
    ControlMode controlMode; // Voltage or current (NB: some chips will generally support one or the other)
    bool        syncEnable;

    // CONFIG register settings
    OscillatorSelect        oscillatorSelect;
    SwitchConfiguration     switchConfiguration;
    OverCurrentDetection    overCurrentDetection;

    // Alarm Register Settings
    AlarmState alarmState;

private:

    void setDefaults();
    
    void setOCThreshold(CurrentThreshold ocThreshold, CommsDriver &commsDriver, int motor );
    void setStallThreshold(CurrentThreshold stallCurrent, CommsDriver &commsDriver, int motor );
    void setStepMode(StepMode stepMode, CommsDriver &commsDriver, int motor );
    void setSyncSelect( SyncSelect syncSelect, bool syncEnable, CommsDriver &commsDriver, int motor );
    void setOscMode(OscillatorSelect oscillatorMode, CommsDriver &commsDriver, int motor );
    void setOCShutdown(OverCurrentDetection overCurrentDetection, CommsDriver &commsDriver, int motor );
    void setSwitchMode(SwitchConfiguration switchMode, CommsDriver &commsDriver, int motor );
    void setAlarmState(AlarmState alarmState, CommsDriver &commsDriver, int motor );
    void setFullSpeed(float stepsPerSecond, CommsDriver &commsDriver, int motor );
    void setLoSpdOpt(bool enable, CommsDriver &commsDriver, int motor );

    CurrentThreshold    getOCThreshold(CommsDriver &commsDriver, int motor);
    CurrentThreshold    getStallThreshold(CommsDriver &commsDriver, int motor);
    StepMode            getStepMode(CommsDriver &commsDriver, int motor );
    SyncSelect          getSyncSelect(CommsDriver &commsDriver, int motor );
    OscillatorSelect    getOscMode(CommsDriver &commsDriver, int motor );
    SwitchConfiguration getSwitchMode(CommsDriver &commsDriver, int motor );
    OverCurrentDetection getOCShutdown(CommsDriver &commsDriver, int motor );
    AlarmState          getAlarmState(CommsDriver &commsDriver, int motor );
    bool                getSyncEnable(CommsDriver &commsDriver, int motor );
    bool                getLoSpdOpt( CommsDriver &commsDriver, int motor );

};

//CommonConfig commonCfgFromString(const std::string &str);
std::string toString(const CommonConfig &cfg);
inline std::ostream& operator<<(std::ostream& os,const CommonConfig &x)
{
    return os << toString(x);
}

std::string getArgument(const std::string &cfg , const std::string &marker);
void tryGetArgumentAsInt(const std::string &cfg, const std::string &marker, int &value);

template <typename T> bool tryReadConfig(const std::string &inputStr, const boost::bimap<T,std::string> &mapping, T &value)
{
    // Try and find a matching element
    if (inputStr != "")
    {
        value = mapping.right.at(inputStr);
        return true;
    }
    else
    {
        std::cout << "No valid cfg entry for marker --> " << std::endl;
        return false;
    }
}