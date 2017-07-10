#include "types.h"
#include <assert.h>
#include <sstream>
#include <boost/bimap.hpp>

// Local Helper Function

// If value > 2^bitLength, val = 2^bitLength
uint32_t capMaxValue (uint32_t value , uint8_t bitLength)
{
    const uint32_t maxValue = 1 << bitLength;
    return ((value > maxValue) ? maxValue : value);
}

std::string toMapString(const std::map <int,uint32_t> &values , uint8_t bitLength)
{
     // TODO - trim the value to bitLength
     std::stringstream ss;
     for (const auto element : values)
     {
         ss << "Motor " << element.first << " --> " << (unsigned int) (element.second & (0xFFFFFFFF >> (32-bitLength)))
            << " (0x" << std::hex << (unsigned int) (element.second & (0xFFFFFFFF >> (32-bitLength))) << ")" << std::endl;
     }

     return ss.str();
}

std::string toLineString(uint8_t *buffer , uint8_t length)
{
    std::stringstream ss;
    ss << "[" << std::hex;

    for (int i=0 ; i < length ; ++i)
    {
       if (i!=0) ss << ",";
       ss << "0x" << (int)buffer[i];
    }
    ss << "]" << std::dec;
    
    return ss.str();
}

boost::bimap <CurrentThreshold,std::string> getCurrentThresholdBiMap()
{
    boost::bimap <CurrentThreshold,std::string> map {
        {OCD_TH_375m  , "375 ma"},
        {OCD_TH_750m  , "750 ma"},
        {OCD_TH_1125m , "1125 ma"},
        {OCD_TH_1500m , "1500 ma"},
        {OCD_TH_1875m , "1875 ma"},
        {OCD_TH_2250m , "2250 ma"},
        {OCD_TH_2625m , "2625 ma"},
        {OCD_TH_3000m , "3000 ma"},
        {OCD_TH_3375m , "3375 ma"},
        {OCD_TH_3750m , "3750 ma"},
        {OCD_TH_4125m , "4125 ma"},
        {OCD_TH_4500m , "4500 ma"},
        {OCD_TH_4875m , "4875 ma"},
        {OCD_TH_5250m , "5250 ma"},
        {OCD_TH_5625m , "5625 ma"},
        {OCD_TH_6000m , "6000 ma"}
    };
    return map;
}


std::string toString(CurrentThreshold currentThreshold)
{
    return getCurrentThresholdBiMap().left.at(currentThreshold);
}

boost::bimap<StepMode,std::string> getStepModeBiMap()
{
    boost::bimap<StepMode,std::string> map {

    { STEP_SEL_1    ,  "Full  step"},
    { STEP_SEL_1_2  ,  "Half  step"},
    { STEP_SEL_1_4  ,  "1/4   microstep"},
    { STEP_SEL_1_8  ,  "1/8   microstep"},
    { STEP_SEL_1_16 ,  "1/16  microstep"},
    { STEP_SEL_1_32 ,  "1/32  microstep"},
    { STEP_SEL_1_64 ,  "1/64  microstep"},
    { STEP_SEL_1_128,  "1/128 microstep"}

    };
    return map;
}

std::string toString(StepMode stepMode)
{
    switch (stepMode)
    {
    case STEP_SEL_1    : return "Full  step";
    case STEP_SEL_1_2  : return "Half  step";
    case STEP_SEL_1_4  : return "1/4   microstep";
    case STEP_SEL_1_8  : return "1/8   microstep";
    case STEP_SEL_1_16 : return "1/16  microstep";
    case STEP_SEL_1_32 : return "1/32  microstep";
    case STEP_SEL_1_64 : return "1/64  microstep";
    case STEP_SEL_1_128: return "1/128 microstep";
    default: assert(!"Invalid stepMode");
    }
}

boost::bimap<SyncSelect,std::string> getSyncSelectBiMap()
{
    boost::bimap<SyncSelect,std::string> map {

    { SYNC_SEL_1_2 ,  "SYNC_SEL_1_2"},
    { SYNC_SEL_1   ,  "SYNC_SEL_1"},
    { SYNC_SEL_2   ,  "SYNC_SEL_2"},
    { SYNC_SEL_4   ,  "SYNC_SEL_4"},
    { SYNC_SEL_8   ,  "SYNC_SEL_80"},
    { SYNC_SEL_16  ,  "SYNC_SEL_16"},
    { SYNC_SEL_32  ,  "SYNC_SEL_32"},
    { SYNC_SEL_64  ,  "SYNC_SEL_64"}

    };
    return map;
}

std::string toString(SyncSelect syncSelect)
{
    switch(syncSelect)
    {
    case SYNC_SEL_1_2 : return "SYNC_SEL_1_2";
    case SYNC_SEL_1   : return "SYNC_SEL_1";
    case SYNC_SEL_2   : return "SYNC_SEL_2";
    case SYNC_SEL_4   : return "SYNC_SEL_4";
    case SYNC_SEL_8   : return "SYNC_SEL_80";
    case SYNC_SEL_16  : return "SYNC_SEL_16";
    case SYNC_SEL_32  : return "SYNC_SEL_32";
    case SYNC_SEL_64  : return "SYNC_SEL_64";
    default: assert(!"Invalid argument");
    }
}

AlarmState::AlarmState()
{
    overCurrentEnabled=true;
    thermalShutdownEnabled=true;
    thermalWarningEnabled=true;
    underVoltageEnabled=true;
    stallDetectionAEnabled=true;
    stallDetectionBEnabled=true;
    switchTurnOnEnabled=true;
    badCommandEnabled=true;
}

std::string toString(AlarmState alarmState)
{
    std::stringstream ss;
    ss << "overCurrentEnabled : "     << (alarmState.overCurrentEnabled ? "Yes" : "No") << std::endl;
    ss << "thermalShutdownEnabled : " << (alarmState.thermalShutdownEnabled ? "Yes" : "No") << std::endl;
    ss << "thermalWarningEnabled  : " << (alarmState.thermalWarningEnabled ? "Yes" : "No") << std::endl;
    ss << "underVoltageEnabled    : " << (alarmState.underVoltageEnabled ? "Yes" : "No") << std::endl;
    ss << "stallDetectionAEnabled : " << (alarmState.stallDetectionAEnabled ? "Yes" : "No") << std::endl;
    ss << "stallDetectionBEnabled : " << (alarmState.stallDetectionBEnabled ? "Yes" : "No") << std::endl;
    ss << "switchTurnOnEnabled    : " << (alarmState.switchTurnOnEnabled ? "Yes" : "No") << std::endl;
    ss << "badCommandEnabled      : " << (alarmState.badCommandEnabled ? "Yes" : "No") << std::endl;
    return ss.str();
}

boost::bimap<OscillatorSelect,std::string> getOscillatorSelectBiMap()
{
    boost::bimap<OscillatorSelect,std::string> map
    {

    { CONFIG_INT_16MHZ                ,  "16MHz"},
    { CONFIG_INT_16MHZ_OSCOUT_2MHZ    ,  "16MHz OSCOUT 2MHz"},
    { CONFIG_INT_16MHZ_OSCOUT_4MHZ    ,  "16MHZ_OSCOUT_4MHZ"},
    { CONFIG_INT_16MHZ_OSCOUT_8MHZ    ,  "16MHZ_OSCOUT_8MHZ"},
    { CONFIG_INT_16MHZ_OSCOUT_16MHZ   ,  "16MHZ_OSCOUT_16MHZ"},
    { CONFIG_EXT_8MHZ_XTAL_DRIVE      ,  "8MHZ_XTAL_DRIVE"},
    { CONFIG_EXT_16MHZ_XTAL_DRIVE     ,  "16MHZ_XTAL_DRIVE"},
    { CONFIG_EXT_24MHZ_XTAL_DRIVE     ,  "24MHZ_XTAL_DRIVE"},
    { CONFIG_EXT_32MHZ_XTAL_DRIVE     ,  "32MHZ_XTAL_DRIVE"},
    { CONFIG_EXT_8MHZ_OSCOUT_INVERT   ,  "8MHZ_OSCOUT_INVERT"},
    { CONFIG_EXT_16MHZ_OSCOUT_INVERT  ,  "16MHZ_OSCOUT_INVERT"},
    { CONFIG_EXT_24MHZ_OSCOUT_INVERT  ,  "24MHZ_OSCOUT_INVERT"},
    { CONFIG_EXT_32MHZ_OSCOUT_INVERT  ,  "32MHZ_OSCOUT_INVERT"}

    };
    return map;
}

std::string toString(OscillatorSelect oscillatorSelect)
{
    switch (oscillatorSelect)
    {
    case CONFIG_INT_16MHZ                : return "16MHz";
    case CONFIG_INT_16MHZ_OSCOUT_2MHZ    : return "16MHz OSCOUT 2MHz";
    case CONFIG_INT_16MHZ_OSCOUT_4MHZ    : return "16MHZ_OSCOUT_4MHZ";
    case CONFIG_INT_16MHZ_OSCOUT_8MHZ    : return "16MHZ_OSCOUT_8MHZ";
    case CONFIG_INT_16MHZ_OSCOUT_16MHZ   : return "16MHZ_OSCOUT_16MHZ";
    case CONFIG_EXT_8MHZ_XTAL_DRIVE      : return "8MHZ_XTAL_DRIVE";
    case CONFIG_EXT_16MHZ_XTAL_DRIVE     : return "16MHZ_XTAL_DRIVE";
    case CONFIG_EXT_24MHZ_XTAL_DRIVE     : return "24MHZ_XTAL_DRIVE";
    case CONFIG_EXT_32MHZ_XTAL_DRIVE     : return "32MHZ_XTAL_DRIVE";
    case CONFIG_EXT_8MHZ_OSCOUT_INVERT   : return "8MHZ_OSCOUT_INVERT";
    case CONFIG_EXT_16MHZ_OSCOUT_INVERT  : return "16MHZ_OSCOUT_INVERT";
    case CONFIG_EXT_24MHZ_OSCOUT_INVERT  : return "24MHZ_OSCOUT_INVERT";
    case CONFIG_EXT_32MHZ_OSCOUT_INVERT  : return "32MHZ_OSCOUT_INVERT";
    default: assert(!"Invalid argument");
    }
}

boost::bimap<SwitchConfiguration,std::string> getSwitchConfigurationBiMap()
{
    boost::bimap<SwitchConfiguration,std::string> map
    {

    { CONFIG_SW_HARD_STOP ,  "Hard Stop"},
    { CONFIG_SW_USER ,  "User Stop"}

    };
    return map;
}

std::string toString(SwitchConfiguration switchConfiguration)
{
    switch(switchConfiguration)
    {
    case CONFIG_SW_HARD_STOP : return "Hard Stop";
    case CONFIG_SW_USER : return "User Stop";
    default: assert(!"Invalid argument");
    }
}

boost::bimap<VoltageCompensation,std::string> getVoltageCompensationBiMap()
{
    boost::bimap<VoltageCompensation,std::string> map
    {

    { CONFIG_VS_COMP_DISABLE ,  "VS_COMP_DISABLE"},
    { CONFIG_VS_COMP_ENABLE  ,  "VS_COMP_ENABLE"}

    };
    return map;
}

std::string toString(VoltageCompensation voltageCompensation)
{
    switch(voltageCompensation)
    {
    case CONFIG_VS_COMP_DISABLE : return "VS_COMP_DISABLE";
    case CONFIG_VS_COMP_ENABLE  : return "VS_COMP_ENABLE";
    default: assert(!"Invalid argument");
    }
}

boost::bimap<OverCurrentDetection,std::string> getOverCurrentDetectionBiMap()
{
    boost::bimap<OverCurrentDetection,std::string> map
    {

    { CONFIG_OC_SD_DISABLE ,  "OC_SD_DSIABLE"},
    { CONFIG_OC_SD_ENABLE  ,  "OC_SD_ENABLE"}

    };
    return map;
}

std::string toString(OverCurrentDetection overCurrentDetection)
{
    switch(overCurrentDetection)
    {
    case CONFIG_OC_SD_DISABLE : return "OC_SD_DSIABLE";
    case CONFIG_OC_SD_ENABLE  : return "OC_SD_ENABLE";
    default: assert(!"Invalid argument");
    }
}

boost::bimap<SlewRate,std::string> getSlewRateBiMap()
{
    boost::bimap<SlewRate,std::string> map
    {

    { CONFIG_SR_180V_us ,  "180V/us"},
    { CONFIG_SR_290V_us ,  "290V/us"},
    { CONFIG_SR_530V_us ,  "530V/us"}

    };
    return map;
}

std::string toString(SlewRate slewRate)
{
    switch(slewRate)
    {
    case CONFIG_SR_180V_us : return "180V/us";
    case CONFIG_SR_290V_us : return "290V/us";
    case CONFIG_SR_530V_us : return "530V/us";
    default: assert(!"Invalid argument");
    }
}

boost::bimap<PwmFrequencyMultiplier,std::string> getPwmFrequencyMultiplierBiMap()
{
    boost::bimap<PwmFrequencyMultiplier,std::string> map
    {

    { CONFIG_PWM_MUL_0_625            ,  "PWM_MUL_0_625"},
    { CONFIG_PWM_MUL_0_75             ,  "PWM_MUL_0_75"},
    { CONFIG_PWM_MUL_0_875            ,  "PWM_MUL_0_875"},
    { CONFIG_PWM_MUL_1                ,  "PWM_MUL_1"},
    { CONFIG_PWM_MUL_1_25             ,  "PWM_MUL_1_25"},
    { CONFIG_PWM_MUL_1_5              ,  "PWM_MUL_1_5"},
    { CONFIG_PWM_MUL_1_75             ,  "PWM_MUL_1_75"},
    { CONFIG_PWM_MUL_2                ,  "PWM_MUL_2"}

    };
    return map;
}

std::string toString(PwmFrequencyMultiplier pwmFrequencyMultiplier)
{
    switch(pwmFrequencyMultiplier)
    {
    case CONFIG_PWM_MUL_0_625            : return "PWM_MUL_0_625";
    case CONFIG_PWM_MUL_0_75             : return "PWM_MUL_0_75";
    case CONFIG_PWM_MUL_0_875            : return "PWM_MUL_0_875";
    case CONFIG_PWM_MUL_1                : return "PWM_MUL_1";
    case CONFIG_PWM_MUL_1_25             : return "PWM_MUL_1_25";
    case CONFIG_PWM_MUL_1_5              : return "PWM_MUL_1_5";
    case CONFIG_PWM_MUL_1_75             : return "PWM_MUL_1_75;";
    case CONFIG_PWM_MUL_2                : return "PWM_MUL_2";
    default: assert(!"Invalid argument");
    }
}

boost::bimap<PwmFrequencyDivider,std::string> getPwmFrequencyDividerBiMap()
{
    boost::bimap<PwmFrequencyDivider,std::string> map
    {

    { CONFIG_PWM_DIV_1                ,  "PWM_DIV_1"},
    { CONFIG_PWM_DIV_2                ,  "PWM_DIV_2"},
    { CONFIG_PWM_DIV_3                ,  "PWM_DIV_3"},
    { CONFIG_PWM_DIV_4                ,  "PWM_DIV_4"},
    { CONFIG_PWM_DIV_5                ,  "PWM_DIV_5"},
    { CONFIG_PWM_DIV_6                ,  "PWM_DIV_6"},
    { CONFIG_PWM_DIV_7                ,  "PWM_DIV_7"}

    };
    return map;
}

std::string toString (PwmFrequencyDivider pwmFrequency)
{
    switch(pwmFrequency)
    {
    case CONFIG_PWM_DIV_1                : return "PWM_DIV_1";
    case CONFIG_PWM_DIV_2                : return "PWM_DIV_2";
    case CONFIG_PWM_DIV_3                : return "PWM_DIV_3";
    case CONFIG_PWM_DIV_4                : return "PWM_DIV_4";
    case CONFIG_PWM_DIV_5                : return "PWM_DIV_5";
    case CONFIG_PWM_DIV_6                : return "PWM_DIV_6";
    case CONFIG_PWM_DIV_7                : return "PWM_DIV_7";
    default: assert(!"Invalid argument");
    }
}

std::string toString (MotorSpinDirection motorSpinDirection)
{
    switch(motorSpinDirection)
    {
    case Forward      : return "Forward";
    case Reverse      : return "Reverse";
    default	      : return "";
        //default: assert(!"Invalid argument");
    }
}

std::string toString (MotorStatus motorStatus)
{
    switch(motorStatus)
    {
    case STATUS_MOT_STATUS_STOPPED        : return "Motor Stopped";
    case STATUS_MOT_STATUS_ACCELERATION   : return "Motor Accelerating";
    case STATUS_MOT_STATUS_DECELERATION   : return "Motor Decelerating";
    case STATUS_MOT_STATUS_CONST_SPD      : return "Motor at constant speed";
    default: assert(!"Invalid argument");
    }
}

std::string toString (Status &status)
{
    // General ControllerState
    std::stringstream ss;

    ss << "isHighZ              : " << (status.isHighZ ? "Yes" : "No") << std::endl;
    ss << "isBusy               : " << (status.isBusy ? "Yes" : "No") << std::endl;
    ss << "isSwitchClosed       : " << (status.isSwitchClosed ? "Yes" : "No") << std::endl;
    ss << "switchEventDetected  : " << (status.switchEventDetected ? "Yes" : "No") << std::endl;
    ss << "performedLastCommand : " << (status.performedLastCommand ? "Yes" : "No") << std::endl;
    ss << "lastCommandInvalid   : " << (status.lastCommandInvalid ? "Yes" : "No") << std::endl;

    // Fault States
    ss << "hasThermalWarning    : " << (status.hasThermalWarning ? "Yes" : "No") << std::endl;
    ss << "isInThermalShutdown  : " << (status.isInThermalShutdown ? "Yes" : "No") << std::endl;
    ss << "overCurrentDetected  : " << (status.overCurrentDetected ? "Yes" : "No") << std::endl;
    ss << "stallDetectedPhaseA  : " << (status.stallDetectedPhaseA ? "Yes" : "No") << std::endl;
    ss << "stallDetectedPhaseB  : " << (status.stallDetectedPhaseB ? "Yes" : "No") << std::endl;

    ss << "stepClockActive      : " << (status.stepClockActive ? "Yes" : "No") << std::endl;

    // Motor State
    ss << "Motor Spin Direction : " << status.spinDirection << std::endl;
    //ss << "MotorStatus          : " << toString(status.motorStatus) << std::end;
    return ss.str();

}

// Parse the status
Status
parseStatus(uint16_t statusValue)
{
    Status status;
    status.isHighZ = statusValue & STATUS_HIZ;
    status.isBusy  = statusValue & STATUS_BUSY;
    status.isSwitchClosed = !(statusValue & STATUS_SW_F);
    status.switchEventDetected = statusValue & STATUS_SW_EVN;

    status.performedLastCommand = !(statusValue & STATUS_NOTPERF_CMD);
    status.lastCommandInvalid   = statusValue & STATUS_WRONG_CMD;

    status.hasThermalWarning = !(statusValue & STATUS_TH_WRN);
    status.isInThermalShutdown = !(statusValue & STATUS_TH_SD);
    status.overCurrentDetected = !(statusValue & STATUS_OCD);
    status.stallDetectedPhaseA = statusValue & STATUS_STEP_LOSS_A;
    status.stallDetectedPhaseB = statusValue & STATUS_STEP_LOSS_B;

    status.spinDirection = static_cast <MotorSpinDirection> (statusValue & STATUS_DIR);
    status.motorStatus   = static_cast <MotorStatus> (statusValue & STATUS_MOT_STATUS);

    return status;
}

std::string toString(Command command)
{
    switch (command)
    {
    case SET_PARAM             : return "SET_PARAM";
    case GET_PARAM             : return "GET_PARAM";
    case RUN                   : return "RUN";
    case STEP_CLOCK            : return "STEP_CLOCK";
    case MOVE                  : return "MOVE";
    case GOTO                  : return "GOTO";
    case GOTO_DIR              : return "GOTO_DIR";
    case GO_UNTIL              : return "GO_UNTIL";
    case RELEASE_SW            : return "RELEASE_SW";
    case GO_HOME               : return "GO_HOME";
    case GO_MARK               : return "GO_MARK";
    case RESET_POS             : return "RESET_POS";
    case RESET_DEVICE          : return "RESET_DEVICE";
    case SOFT_STOP             : return "SOFT_STOP";
    case HARD_STOP             : return "HARD_STOP";
    case SOFT_HIZ              : return "SOFT_HIZ";
    case HARD_HIZ              : return "HARD_HIZ";
    case GET_STATUS            : return "GET_STATUS";
    default: assert(!"Invalid argument");
    }
}

std::string toString(ParamRegister paramRegister)
{
    switch (paramRegister)
    {
    case ABS_POS    : return "ABS_POS";
    case EL_POS     : return "EL_POS";
    case MARK       : return "MARK";
    case SPEED      : return "SPEED";
    case ACC        : return "ACC";
    case DECEL      : return "DECEL";
    case MAX_SPEED  : return "MAX_SPEED";
    case MIN_SPEED  : return "MIN_SPEED";
    case FS_SPD     : return "FS_SPD";
    case KVAL_HOLD  : return "KVAL_HOLD";
    case KVAL_RUN   : return "KVAL_RUN";
    case KVAL_ACC   : return "KVAL_ACC";
    case KVAL_DEC   : return "KVAL_DEC";
    case INT_SPD    : return "INT_SPD";
    case ST_SLP     : return "ST_SLP";
    case FN_SLP_ACC : return "FN_SLP_ACC";
    case FN_SLP_DEC : return "FN_SLP_DEC";
    case K_THERM    : return "K_THERM";
    case ADC_OUT    : return "ADC_OUT";
    case OCD_TH     : return "OCD_TH";
    case STALL_TH   : return "STALL_TH";
    case STEP_MODE  : return "STEP_MODE";
    case ALARM_EN   : return "ALARM_EN";
    case CONFIG     : return "CONFIG";
    case STATUS     : return "STATUS";
    default : return "Unknown paramRgister (" + std::to_string((int) paramRegister)+")";
    }
}

std::string toString(const Config &cfg)
{
    std::stringstream ss;

    ss << "BackEmfConfig           : " << std::endl << cfg.backEmfConfig << std::endl;
    ss << "FullStepThresholdSpeed  : " << cfg.fullStepThresholdSpeed << " steps/s" << std::endl;
    ss << "ThermalDriftCoefficient : " << cfg.thermalDriftCoefficient << std::endl;
    ss << "OverCurrentThreshold    : " << cfg.overCurrentThreshold << std::endl;
    ss << "StallThreshold          : " << cfg.stallThreshold << std::endl;
    ss << "StepMode                : " << cfg.stepMode << std::endl;
    ss << "SyncSelect              : " << cfg.syncSelect << std::endl;
    ss << "SyncEnable              : " << (cfg.syncEnable ? "Yes" : "No") << std::endl;
    ss << "OscillatorSelect        : " << cfg.oscillatorSelect << std::endl;
    ss << "SwitchConfiguration     : " << cfg.switchConfiguration << std::endl;
    ss << "OverCurrentDetection    : " << cfg.overCurrentDetection << std::endl;
    ss << "SlewRate                : " << cfg.slewRate << std::endl;
    ss << "VoltageCompensation     : " << cfg.voltageCompensation << std::endl;
    ss << "PwmFrequencyMultiplier  : " << cfg.pwmFrequencyMultiplier << std::endl;
    ss << "PwmFrequencyDivider     : " << cfg.pwmFrequencyDivider << std::endl;
    ss << "AlarmState              : " << (int)cfg.alarmState << std::endl;

    return ss.str();
}

void tryReadAndSetWithDefault(const std::string &marker , T &vvariable ,

Config cfgFromString(const std::string &str)
{
    // Create a default config
    Config cfg;

    // TODO - parse backEmfConfig

    // Handle standard configuration parameters
    if (str.find("BackEmfConfig           : " ) != std::string::npos) std::endl ) != std::string::npos) cfg.backEmfConfig ) != std::string::npos) std::endl;
    if (str.find("FullStepThresholdSpeed  : " ) != std::string::npos) cfg.fullStepThresholdSpeed ) != std::string::npos) " steps/s" ) != std::string::npos) std::endl;
    if (str.find("ThermalDriftCoefficient : " ) != std::string::npos) cfg.thermalDriftCoefficient ) != std::string::npos) std::endl;
    if (str.find("OverCurrentThreshold    : " ) != std::string::npos) cfg.overCurrentThreshold ) != std::string::npos) std::endl;
    if (str.find("StallThreshold          : " ) != std::string::npos) cfg.stallThreshold ) != std::string::npos) std::endl;
    if (str.find("StepMode                : " ) != std::string::npos) cfg.stepMode ) != std::string::npos) std::endl;
    if (str.find("SyncSelect              : " ) != std::string::npos) cfg.syncSelect ) != std::string::npos) std::endl;
    if (str.find("SyncEnable              : " ) != std::string::npos) (cfg.syncEnable ? "Yes" : "No") ) != std::string::npos) std::endl;
    if (str.find("OscillatorSelect        : " ) != std::string::npos) cfg.oscillatorSelect ) != std::string::npos) std::endl;
    if (str.find("SwitchConfiguration     : " ) != std::string::npos) cfg.switchConfiguration ) != std::string::npos) std::endl;
    if (str.find("OverCurrentDetection    : " ) != std::string::npos) cfg.overCurrentDetection ) != std::string::npos) std::endl;
    if (str.find("SlewRate                : " ) != std::string::npos) cfg.slewRate ) != std::string::npos) std::endl;
    if (str.find("VoltageCompensation     : " ) != std::string::npos) cfg.voltageCompensation ) != std::string::npos) std::endl;
    if (str.find("PwmFrequencyMultiplier  : " ) != std::string::npos) cfg.pwmFrequencyMultiplier ) != std::string::npos) std::endl;
    if (str.find("PwmFrequencyDivider     : " ) != std::string::npos) cfg.pwmFrequencyDivider ) != std::string::npos) std::endl;
    if (str.find("AlarmState              : " ) != std::string::npos) (int)cfg.alarmState ) != std::string::npos) std::endl;

}

std::string toString(const ProfileCfg &profileCfg)
{
    std::stringstream ss;
    ss << "acceleration = " << profileCfg.acceleration << " steps/s^2" << std::endl;
    ss << "deceleration = " << profileCfg.deceleration << " steps/s^2" << std::endl;
    ss << "maxSpeed     = " << profileCfg.maxSpeed     << " steps/s"   << std::endl;
    ss << "minSpeed     = " << profileCfg.minSpeed     << " steps/s"   << std::endl;
    return ss.str();
}
