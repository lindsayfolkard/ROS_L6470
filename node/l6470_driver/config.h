#pragma once

#include "types.h"
#include "commsdriver.h"

// Abstract Classes for passing configs around
class AbstractConfig {
public:
    virtual ~AbstractConfig() = 0;
    virtual void set(CommsDriver &commsDriver, int motor) = 0;
    virtual void readFromFile(const std::string &filePath) = 0;
    virtual void unitTest(CommsDriver &commsDriver, int motor) = 0;
};

class WriteableConfig {
public:
    virtual ~WriteableConfig() = 0;
    virtual void writeToFile(const std::string &cfgFilePath) = 0;
};

enum MotorDriverType
{
    PowerStep01,
    L6470, // not checked as of yet
    L6472,  // not checked as of yet
    Simulator
};
std::string toString(MotorDriverType motorDriverType);
inline std::ostream& operator<<(std::ostream& os, MotorDriverType x)
{
    return os << toString(x);
}
MotorDriverType motorDriverTypeFromString(const std::string &str);

///
/// \brief The CfgFile struct
/// \abstract Contains links to required config files for a particular stepper motor
struct CfgFile
{
    CfgFile(const std::string motorModel, const std::string &stepperMotorFile, const std::string &customConfigFile="", const std::string voltageModeConfigFile="", const std::string currentModeConfigFile="") :
        motorModel_(motorModel),
        stepperMotorFile_(stepperMotorFile),
        commonConfigFile_(customConfigFile),
        voltageModeConfigFile_(voltageModeConfigFile),
        currentModeConfigFile_(currentModeConfigFile)
        {}

    std::string motorModel_;
    std::string stepperMotorFile_;
    std::string commonConfigFile_;      // if empty, default commonConfig is used
    std::string voltageModeConfigFile_; // if empty, then voltage mode config is derived from the stepper motor properties
    std::string currentModeConfigFile_; // if not empty, then the current mode config is derived from the stepper motor properties ???
};

struct OverallCfg
{
    OverallCfg(const std::string &filePath);
    OverallCfg(const std::vector<CfgFile> &cfgFiles,
               MotorDriverType             controllerType,
               CommsDebugLevel             commsDebugLevel = CommsDebugNothing,
               int                         spiBus = 0);

    void writeToFile(const std::string &baseFile);

    std::vector<CfgFile>     cfgFiles_;
    MotorDriverType          controllerType_;
    CommsDebugLevel          commsDebugLevel_;
    int                      spiBus_;
};
std::string toString(const OverallCfg &cfg);
inline std::ostream& operator<<(std::ostream& os,const OverallCfg &x)
{
    return os << toString(x);
}

enum Colour {
    Red,
    Blue,
    Green,
    Yellow,
    Orange
};

std::string addColour(const std::string &str, Colour colour);

class CurrentModeCfg : public AbstractConfig,
                       public WriteableConfig
{

public:

    virtual void set(CommsDriver &commsDriver, int motor) override;
    virtual void readFromFile(const std::string &filePath) override;
    virtual void writeToFile(const std::string &filePath) override;
    virtual void unitTest(CommsDriver &commsDriver, int motor) override;

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
    virtual void unitTest(CommsDriver &commsDriver, int motor) override;

    uint8_t holdingKVal;
    uint8_t constantSpeedKVal;
    uint8_t accelStartingKVal;
    uint8_t decelStartingKVal;

    uint32_t intersectSpeed;
    uint32_t startSlope;
    uint32_t accelFinalSlope;
    uint32_t decelFinalSlope;

    ThermalDriftCompensation thermalDriftCompensation;
    SlewRate                 slewRate;
    VoltageCompensation      voltageCompensation;
    PwmFrequencyMultiplier   pwmFrequencyMultiplier;
    PwmFrequencyDivider      pwmFrequencyDivider;
    bool                     enableLowSpeedOptimisation;

private:

    void setDefaults();
    
    // probs can just be external methods in their own namespace but fuck it
    static void setAccKVAL(uint8_t kvalInput, CommsDriver &commsDriver, int motor );
    static void setDecKVAL(uint8_t kvalInput, CommsDriver &commsDriver, int motor );
    static void setRunKVAL(uint8_t kvalInput, CommsDriver &commsDriver, int motor );
    static void setHoldKVAL(uint8_t kvalInput, CommsDriver &commsDriver, int motor );
    static void setVoltageComp(VoltageCompensation vsCompMode, CommsDriver &commsDriver, int motor );
    static void setPWMFreq(PwmFrequencyDivider divider, PwmFrequencyMultiplier multiplier,  CommsDriver &commsDriver, int motor );
    static void setPWMFreqDivider(PwmFrequencyDivider divider, CommsDriver &commsDriver, int motor);
    static void setPWMFreqMultiplier(PwmFrequencyMultiplier multiplier, CommsDriver &commsDriver, int motor);
    static void setSlewRate(SlewRate slewRate, CommsDriver &commsDriver, int motor );
    static void setThermalDriftCompensation(ThermalDriftCompensation thermalDriftCompensation, CommsDriver &commsDriver, int motor);

    static uint8_t getAccKVAL(CommsDriver &commsDriver, int motor );
    static uint8_t getDecKVAL(CommsDriver &commsDriver, int motor );
    static uint8_t getRunKVAL(CommsDriver &commsDriver, int motor );
    static uint8_t getHoldKVAL(CommsDriver &commsDriver, int motor );

    static ThermalDriftCompensation getThermalDriftCompensation(CommsDriver &commsDriver, int motor);
    static PwmFrequencyDivider	    getPWMFreqDivisor(CommsDriver &commsDriver, int motor );
    static PwmFrequencyMultiplier   getPWMFreqMultiplier(CommsDriver &commsDriver, int motor );
    static VoltageCompensation	    getVoltageComp(CommsDriver &commsDriver, int motor );
    static SlewRate                 getSlewRate(CommsDriver &commsDriver, int motor );
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
    CommonConfig(CommsDriver &commsDriver , int motor);
    
    virtual void set(CommsDriver &commsDriver, int motor) override;
    virtual void readFromFile(const std::string &file) override;
    virtual void writeToFile(const std::string &cfgFilePath) override;
    virtual void unitTest(CommsDriver &commsDriver, int motor) override;

    int fullStepThresholdSpeed;

    // Current Thresholds
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

    // Gate Configuration settings (PowerStep01 only)
    GateConfig1 gateConfig1;
    GateConfig2 gateConfig2;

private:

    void setDefaults();
    
    static void setOCThreshold(CurrentThreshold ocThreshold, CommsDriver &commsDriver, int motor );
    static void setStallThreshold(CurrentThreshold stallCurrent, CommsDriver &commsDriver, int motor );
    static void setStepMode(StepMode stepMode, CommsDriver &commsDriver, int motor );
    static void setSyncSelect( SyncSelect syncSelect, bool syncEnable, CommsDriver &commsDriver, int motor );
    static void setOscMode(OscillatorSelect oscillatorMode, CommsDriver &commsDriver, int motor );
    static void setOCShutdown(OverCurrentDetection overCurrentDetection, CommsDriver &commsDriver, int motor );
    static void setSwitchMode(SwitchConfiguration switchMode, CommsDriver &commsDriver, int motor );
    static void setAlarmState(AlarmState alarmState, CommsDriver &commsDriver, int motor );
    static void setFullSpeed(float stepsPerSecond, CommsDriver &commsDriver, int motor );
    static void setLoSpdOpt(bool enable, CommsDriver &commsDriver, int motor );
    static void setControlMode (ControlMode controlMode, CommsDriver &commsDriver, int motor );
    static void setGateConfig1(const GateConfig1 &gateConfig1, CommsDriver &commsDriver, int motor);
    static void setGateConfig2(const GateConfig2 &gateConfig2, CommsDriver &commsDriver, int motor);

    static CurrentThreshold     getOCThreshold(CommsDriver &commsDriver, int motor);
    static CurrentThreshold     getStallThreshold(CommsDriver &commsDriver, int motor);
    static StepMode             getStepMode(CommsDriver &commsDriver, int motor );
    static SyncSelect           getSyncSelect(CommsDriver &commsDriver, int motor );
    static OscillatorSelect     getOscMode(CommsDriver &commsDriver, int motor );
    static SwitchConfiguration  getSwitchMode(CommsDriver &commsDriver, int motor );
    static OverCurrentDetection getOCShutdown(CommsDriver &commsDriver, int motor );
    static AlarmState           getAlarmState(CommsDriver &commsDriver, int motor );
    static bool                 getSyncEnable(CommsDriver &commsDriver, int motor );
    static bool                 getLoSpdOpt(CommsDriver &commsDriver, int motor );
    static int                  getFullSpeed(CommsDriver &commsDriver, int motor);
    static ControlMode          getControlMode(CommsDriver &commsDriver, int motor);
    static GateConfig1          getGateConfig1(CommsDriver &commsDriver, int motor);
    static GateConfig2          getGateConfig2(CommsDriver &commsDriver, int motor);

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
