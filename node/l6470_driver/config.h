#include "types.h"
#include "commsdriver.h"

struct CurrentModeCfg
{
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
    OverCurrentDetection  overCurrentDetection;
    OscillatorSelect      oscillatorSelect;
    bool                  enableTorqueRegulation;
    bool                  externalClockEnabled;
};

///
/// \brief The BackEmfConfig struct
/// contains all information that is needed to
/// configure a motor to be used with a controller
///
struct VoltageModeCfg
{
    uint8_t holdingKVal;
    uint8_t constantSpeedKVal;
    uint8_t accelStartingKVal;
    uint8_t decelStartingKVal;

    uint32_t intersectSpeed;
    uint32_t startSlope;
    uint32_t accelFinalSlope;
    uint32_t decelFinalSlope;

    // Config Register
    OscillatorSelect       oscillatorSelect;
    SwitchConfiguration    switchConfiguration;
    OverCurrentDetection   overCurrentDetection;
    //SlewRate               slewRate;
    VoltageCompensation    voltageCompensation;
    PwmFrequencyMultiplier pwmFrequencyMultiplier;
    PwmFrequencyDivider    pwmFrequencyDivider;
};
std::string toString(const VoltageModeCfg &backEmfConfig);
inline std::ostream& operator<<(std::ostream& os,const VoltageModeCfg &x)
{
    return os << toString(x);
}

VoltageModeCfg getBackEmfConfigFromString(const std::string &cfg);

// General Static Config (meant to be set once at the start and then not really again)
// NB: valid parameters are always positive. A negative parameter is interpreted as do not set/read.
struct Config
{
    // Very Important w.r.t smooth motor driving
    // See motor.h for more information
    boost::optional<VoltageModeCfg> voltageModeConfig;
    boost::optional<CurrentModeCfg> currentModeConfig;

    int fullStepThresholdSpeed;
    int thermalDriftCoefficient;

    CurrentThreshold overCurrentThreshold;
    CurrentThreshold stallThreshold;

    // STEP_MODE register settings
    StepMode    stepMode;
    SyncSelect  syncSelect;
    bool        syncEnable;

    // CONFIG register settings
    OscillatorSelect        oscillatorSelect;
    SwitchConfiguration     switchConfiguration;
    OverCurrentDetection    overCurrentDetection;
    SlewRate                slewRate;
    VoltageCompensation     voltageCompensation;
    PwmFrequencyMultiplier  pwmFrequencyMultiplier;
    PwmFrequencyDivider     pwmFrequencyDivider;

    // Alarm Register Settings
    AlarmState alarmState;
};

Config cfgFromString(const std::string &str);
std::string toString(const Config &cfg);
inline std::ostream& operator<<(std::ostream& os,const Config &x)
{
    return os << toString(x);
}

std::string getArgument(const std::string &cfg , const std::string &marker);
void tryGetArgumentAsInt(const std::string &cfg, const std::string &marker, int &value);

template <typename T> bool tryReadConfig(const std::string &cfg , const std::string &marker, const boost::bimap<T,std::string> &mapping, T &value)
{
    std::string argument = getArgument(cfg,marker);

    // Try and find a matching element
    if (argument != "")
    {
        value = mapping.right.at(argument);
        return true;
    }
    else
    {
        std::cout << "No valid cfg entry for marker --> " << marker << std::endl;
        return false;
    }
}

void setLoSpdOpt(bool enable , CommsDriver &commsDriver , int motor );
void setSyncSelect( SyncSelect syncSelect , bool syncEnable ,  CommsDriver &commsDriver , int motor );
void setStepMode(StepMode stepMode,  CommsDriver &commsDriver , int motor );
void setFullSpeed(float stepsPerSecond,  CommsDriver &commsDriver , int motor );

void setOCThreshold(CurrentThreshold ocThreshold ,  CommsDriver &commsDriver , int motor );
void setStallThreshold(CurrentThreshold stallCurrent , CommsDriver &commsDriver, int motor );
void setPWMFreq(PwmFrequencyDivider divider, PwmFrequencyMultiplier multiplier,  CommsDriver &commsDriver, int motor );
void setSlewRate(SlewRate slewRate,  CommsDriver &commsDriver, int motor );
void setOCShutdown(OverCurrentDetection overCurrentDetection,  CommsDriver &commsDriver, int motor );
void setVoltageComp(VoltageCompensation vsCompMode, CommsDriver &commsDriver, int motor );
void setSwitchMode(SwitchConfiguration switchMode, CommsDriver &commsDriver, int motor );
void setOscMode(OscillatorSelect oscillatorMode, CommsDriver &commsDriver, int motor );
void setAlarmState(AlarmState alarmState, CommsDriver &commsDriver, int motor );

void setAccKVAL(uint8_t kvalInput, CommsDriver &commsDriver, int motor );
void setDecKVAL(uint8_t kvalInput, CommsDriver &commsDriver, int motor );
void setRunKVAL(uint8_t kvalInput, CommsDriver &commsDriver, int motor );
void setHoldKVAL(uint8_t kvalInput, CommsDriver &commsDriver, int motor );

bool getLoSpdOpt( CommsDriver &commsDriver, int motor );


StepMode		   getStepMode(CommsDriver &commsDriver, int motor );
SyncSelect		   getSyncSelect(CommsDriver &commsDriver, int motor );
bool                       getSyncEnable(CommsDriver &commsDriver, int motor );
CurrentThreshold	   getOCThreshold(CommsDriver &commsDriver, int motor );
CurrentThreshold	   getStallThreshold(CommsDriver &commsDriver, int motor );
PwmFrequencyDivider	   getPWMFreqDivisor(CommsDriver &commsDriver, int motor );
PwmFrequencyMultiplier     getPWMFreqMultiplier(CommsDriver &commsDriver, int motor );
SlewRate		   getSlewRate(CommsDriver &commsDriver, int motor );
OverCurrentDetection       getOCShutdown(CommsDriver &commsDriver, int motor );
VoltageCompensation	   getVoltageComp(CommsDriver &commsDriver, int motor );
SwitchConfiguration	   getSwitchMode(CommsDriver &commsDriver, int motor );
OscillatorSelect	   getOscMode(CommsDriver &commsDriver, int motor );
AlarmState		   getAlarmState(CommsDriver &commsDriver, int motor );

uint8_t getAccKVAL(CommsDriver &commsDriver, int motor );
uint8_t getDecKVAL(CommsDriver &commsDriver, int motor );
uint8_t getRunKVAL(CommsDriver &commsDriver, int motor );
uint8_t getHoldKVAL(CommsDriver &commsDriver, int motor );
