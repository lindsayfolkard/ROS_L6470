#include "types.h"
#include <assert.h>
#include <sstream>
#include <boost/bimap.hpp>
#include "motor.h"

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

std::string toLineString(const uint8_t *buffer , uint8_t length)
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

std::string toLineString(const std::vector<uint8_t> &data)
{
    return toLineString(&data[0],data.size());
}

template <typename L, typename R>
boost::bimap<L, R>
makeBiMap(std::initializer_list<typename boost::bimap<L, R>::value_type> list)
{
    return boost::bimap<L, R>(list.begin(), list.end());
}

boost::bimap <CurrentThreshold,std::string> getCurrentThresholdBiMap()
{
    boost::bimap <CurrentThreshold,std::string> map  = makeBiMap<CurrentThreshold,std::string>(
    {
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
                    {OCD_TH_6000m , "6000 ma"},
                    {OCD_TH_6375m , "6375 ma"},
                    {OCD_TH_6750m , "6750 ma"},
                    {OCD_TH_7125m , "7125 ma"},
                    {OCD_TH_7500m , "7500 ma"},
                    {OCD_TH_7875m , "7875 ma"},
                    {OCD_TH_8250m , "8250 ma"},
                    {OCD_TH_8625m , "8625 ma"},
                    {OCD_TH_9000m , "9000 ma"},
                    {OCD_TH_9375m , "9375 ma"},
                    {OCD_TH_9750m , "9750 ma"},
                    {OCD_TH_10125m , "10125 ma"},
                    {OCD_TH_10500m , "10500 ma"},
                    {OCD_TH_10875m , "10875 ma"},
                    {OCD_TH_11250m , "11250 ma"},
                    {OCD_TH_11625m , "11625 ma"},
                    {OCD_TH_12000m , "12000 ma"}/*,
                    {OCD_TH_12375m , "12375 ma"},
                    {OCD_TH_12750m , "12750 ma"},
                    {OCD_TH_13125m , "13125 ma"},
                    {OCD_TH_13500m , "13500 ma"}*/

                });
    return map;
}


std::string toString(CurrentThreshold currentThreshold)
{
    return getCurrentThresholdBiMap().left.at(currentThreshold);
}

boost::bimap<StepMode,std::string> getStepModeBiMap()
{
    boost::bimap<StepMode,std::string> map  = makeBiMap<StepMode,std::string>({

    { STEP_SEL_1    ,  "Full  step"},
    { STEP_SEL_1_2  ,  "Half  step"},
    { STEP_SEL_1_4  ,  "1/4   microstep"},
    { STEP_SEL_1_8  ,  "1/8   microstep"},
    { STEP_SEL_1_16 ,  "1/16  microstep"},
    { STEP_SEL_1_32 ,  "1/32  microstep"},
    { STEP_SEL_1_64 ,  "1/64  microstep"},
    { STEP_SEL_1_128,  "1/128 microstep"}

    });
    return map;
}

boost::bimap<ControlMode,std::string> getControlModeBiMap()
{
    boost::bimap<ControlMode,std::string> map = makeBiMap<ControlMode,std::string>
    ({

    { VoltageControlMode, "VoltageControlMode"},
    { CurrentControlMode, "CurrentControlMode"}

    });
    return map;
}

std::string toString(ControlMode controlMode)
{
    return getControlModeBiMap().left.at(controlMode);
}

std::string toString(StepMode stepMode)
{
    return getStepModeBiMap().left.at(stepMode);
}

boost::bimap<SyncSelect,std::string> getSyncSelectBiMap()
{
    boost::bimap<SyncSelect,std::string> map = makeBiMap<SyncSelect,std::string>({

    { SYNC_SEL_1_2 ,  "SYNC_SEL_1_2"},
    { SYNC_SEL_1   ,  "SYNC_SEL_1"},
    { SYNC_SEL_2   ,  "SYNC_SEL_2"},
    { SYNC_SEL_4   ,  "SYNC_SEL_4"},
    { SYNC_SEL_8   ,  "SYNC_SEL_80"},
    { SYNC_SEL_16  ,  "SYNC_SEL_16"},
    { SYNC_SEL_32  ,  "SYNC_SEL_32"},
    { SYNC_SEL_64  ,  "SYNC_SEL_64"}

    });
    return map;
}

std::string toString(SyncSelect syncSelect)
{
    return getSyncSelectBiMap().left.at(syncSelect);
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
    ss << "OverCurrentEnabled     : " << (alarmState.overCurrentEnabled ? "Yes" : "No") << std::endl;
    ss << "ThermalShutdownEnabled : " << (alarmState.thermalShutdownEnabled ? "Yes" : "No") << std::endl;
    ss << "ThermalWarningEnabled  : " << (alarmState.thermalWarningEnabled ? "Yes" : "No") << std::endl;
    ss << "UnderVoltageEnabled    : " << (alarmState.underVoltageEnabled ? "Yes" : "No") << std::endl;
    ss << "StallDetectionAEnabled : " << (alarmState.stallDetectionAEnabled ? "Yes" : "No") << std::endl;
    ss << "StallDetectionBEnabled : " << (alarmState.stallDetectionBEnabled ? "Yes" : "No") << std::endl;
    ss << "SwitchTurnOnEnabled    : " << (alarmState.switchTurnOnEnabled ? "Yes" : "No") << std::endl;
    ss << "BadCommandEnabled      : " << (alarmState.badCommandEnabled ? "Yes" : "No") << std::endl;
    return ss.str();
}

std::string getArgument(const std::string &cfg , const std::string &marker)
{
    // Try to find the marker in the cfg
    size_t markerPosition = cfg.find(marker);

    if (markerPosition == std::string::npos) return "";

    size_t colonPosition = cfg.find(':',markerPosition);

    if (colonPosition == std::string::npos) return "";

    size_t endLinePosition = cfg.find('\n',colonPosition+1);

    if (endLinePosition == std::string::npos) return "";

    // Get the string (skip initial whitespace)
    std::string element = cfg.substr(colonPosition+1,endLinePosition);
    // trim trailing whitespace .... TODO

    return element;
}

void tryGetArgumentAsInt(const std::string &cfg, const std::string &marker, int &value)
{
    std::string argument = getArgument(cfg,marker);
    if (argument != "") value = std::stoi(argument);
    else
    {
        std::cout << "No value found for marker "<< marker;
    }
}

void tryExtractAlarmStateFromElement(const std::string &cfg, const std::string &marker , bool &cfgValue)
{
    std::string argument = getArgument(cfg,marker);

    if (argument == "") return;

    if (argument.find("No")!=std::string::npos)
        cfgValue = false;
    else
        cfgValue = true;
}

AlarmState getAlarmStateFromString(const std::string &str)
{
    AlarmState alarmState;
    tryExtractAlarmStateFromElement(str,"OverCurrentEnabled",alarmState.overCurrentEnabled);
    tryExtractAlarmStateFromElement(str,"ThermalShutdownEnabled",alarmState.thermalShutdownEnabled);
    tryExtractAlarmStateFromElement(str,"ThermalWarningEnabled",alarmState.thermalWarningEnabled);
    tryExtractAlarmStateFromElement(str,"UnderVoltageEnabled",alarmState.underVoltageEnabled);
    tryExtractAlarmStateFromElement(str,"StallDetectionAEnabled",alarmState.stallDetectionAEnabled);
    tryExtractAlarmStateFromElement(str,"StallDetectionBEnabled",alarmState.stallDetectionBEnabled);
    tryExtractAlarmStateFromElement(str,"SwitchTurnOnEnabled",alarmState.switchTurnOnEnabled);
    tryExtractAlarmStateFromElement(str,"BadCommandEnabled",alarmState.badCommandEnabled);
    return alarmState;
}

boost::bimap<OscillatorSelect,std::string> getOscillatorSelectBiMap()
{
    boost::bimap<OscillatorSelect,std::string> map = makeBiMap<OscillatorSelect,std::string>
    ({

    { CONFIG_INT_16MHZ                ,  "INT_16MHz"},
    { CONFIG_INT_16MHZ_OSCOUT_2MHZ    ,  "INT_16MHz OSCOUT 2MHz"},
    { CONFIG_INT_16MHZ_OSCOUT_4MHZ    ,  "INT_16MHZ_OSCOUT_4MHZ"},
    { CONFIG_INT_16MHZ_OSCOUT_8MHZ    ,  "INT_16MHZ_OSCOUT_8MHZ"},
    { CONFIG_INT_16MHZ_OSCOUT_16MHZ   ,  "INT_16MHZ_OSCOUT_16MHZ"},
    { CONFIG_EXT_8MHZ_XTAL_DRIVE      ,  "EXT_8MHZ_XTAL_DRIVE"},
    { CONFIG_EXT_16MHZ_XTAL_DRIVE     ,  "EXT_16MHZ_XTAL_DRIVE"},
    { CONFIG_EXT_24MHZ_XTAL_DRIVE     ,  "EXT_24MHZ_XTAL_DRIVE"},
    { CONFIG_EXT_32MHZ_XTAL_DRIVE     ,  "EXT_32MHZ_XTAL_DRIVE"},
    { CONFIG_EXT_8MHZ_OSCOUT_INVERT   ,  "EXT_8MHZ_OSCOUT_INVERT"},
    { CONFIG_EXT_16MHZ_OSCOUT_INVERT  ,  "EXT_16MHZ_OSCOUT_INVERT"},
    { CONFIG_EXT_24MHZ_OSCOUT_INVERT  ,  "EXT_24MHZ_OSCOUT_INVERT"},
    { CONFIG_EXT_32MHZ_OSCOUT_INVERT  ,  "EXT_32MHZ_OSCOUT_INVERT"}

    });
    return map;
}

std::string toString(OscillatorSelect oscillatorSelect)
{
    return getOscillatorSelectBiMap().left.at(oscillatorSelect);
}

boost::bimap<SwitchConfiguration,std::string> getSwitchConfigurationBiMap()
{
    boost::bimap<SwitchConfiguration,std::string> map = makeBiMap <SwitchConfiguration,std::string>
    ({

    { CONFIG_SW_HARD_STOP ,  "Hard Stop"},
    { CONFIG_SW_USER ,  "User Stop"}

    });
    return map;
}

std::string toString(SwitchConfiguration switchConfiguration)
{
    return getSwitchConfigurationBiMap().left.at(switchConfiguration);
}

boost::bimap<VoltageCompensation,std::string> getVoltageCompensationBiMap()
{
    boost::bimap<VoltageCompensation,std::string> map = makeBiMap<VoltageCompensation,std::string>
    ({

    { CONFIG_VS_COMP_DISABLE ,  "VS_COMP_DISABLE"},
    { CONFIG_VS_COMP_ENABLE  ,  "VS_COMP_ENABLE"}

    });
    return map;
}

std::string toString(VoltageCompensation voltageCompensation)
{
    return getVoltageCompensationBiMap().left.at(voltageCompensation);
}

boost::bimap<OverCurrentDetection,std::string> getOverCurrentDetectionBiMap()
{
    boost::bimap<OverCurrentDetection,std::string> map = makeBiMap<OverCurrentDetection,std::string>
    ({

    { CONFIG_OC_SD_DISABLE ,  "OC_SD_DISABLE"},
    { CONFIG_OC_SD_ENABLE  ,  "OC_SD_ENABLE"}

    });
    return map;
}

std::string toString(OverCurrentDetection overCurrentDetection)
{
    return getOverCurrentDetectionBiMap().left.at(overCurrentDetection);
}

boost::bimap<SlewRate,std::string> getSlewRateBiMap()
{
    boost::bimap<SlewRate,std::string> map = makeBiMap<SlewRate,std::string>
    ({

    { CONFIG_SR_180V_us ,  "180V/us"},
    { CONFIG_SR_290V_us ,  "290V/us"},
    { CONFIG_SR_530V_us ,  "530V/us"}

    });
    return map;
}

std::string toString(SlewRate slewRate)
{
    return getSlewRateBiMap().left.at(slewRate);
}

boost::bimap<PwmFrequencyMultiplier,std::string> getPwmFrequencyMultiplierBiMap()
{
    boost::bimap<PwmFrequencyMultiplier,std::string> map = makeBiMap<PwmFrequencyMultiplier,std::string>
    ({

    { CONFIG_PWM_MUL_0_625            ,  "PWM_MUL_0_625"},
    { CONFIG_PWM_MUL_0_75             ,  "PWM_MUL_0_75"},
    { CONFIG_PWM_MUL_0_875            ,  "PWM_MUL_0_875"},
    { CONFIG_PWM_MUL_1                ,  "PWM_MUL_1"},
    { CONFIG_PWM_MUL_1_25             ,  "PWM_MUL_1_25"},
    { CONFIG_PWM_MUL_1_5              ,  "PWM_MUL_1_5"},
    { CONFIG_PWM_MUL_1_75             ,  "PWM_MUL_1_75"},
    { CONFIG_PWM_MUL_2                ,  "PWM_MUL_2"}

    });
    return map;
}

std::string toString(PwmFrequencyMultiplier pwmFrequencyMultiplier)
{
    return getPwmFrequencyMultiplierBiMap().left.at(pwmFrequencyMultiplier);
}

boost::bimap<PwmFrequencyDivider,std::string> getPwmFrequencyDividerBiMap()
{
    boost::bimap<PwmFrequencyDivider,std::string> map = makeBiMap<PwmFrequencyDivider,std::string>
    ({

    { CONFIG_PWM_DIV_1                ,  "PWM_DIV_1"},
    { CONFIG_PWM_DIV_2                ,  "PWM_DIV_2"},
    { CONFIG_PWM_DIV_3                ,  "PWM_DIV_3"},
    { CONFIG_PWM_DIV_4                ,  "PWM_DIV_4"},
    { CONFIG_PWM_DIV_5                ,  "PWM_DIV_5"},
    { CONFIG_PWM_DIV_6                ,  "PWM_DIV_6"},
    { CONFIG_PWM_DIV_7                ,  "PWM_DIV_7"}

    });
    return map;
}

std::string toString (PwmFrequencyDivider pwmFrequency)
{
    return getPwmFrequencyDividerBiMap().left.at(pwmFrequency);
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

std::string toString(const ProfileCfg &profileCfg)
{
    std::stringstream ss;
    ss << "acceleration = " << profileCfg.acceleration << " steps/s^2" << std::endl;
    ss << "deceleration = " << profileCfg.deceleration << " steps/s^2" << std::endl;
    ss << "maxSpeed     = " << profileCfg.maxSpeed     << " steps/s"   << std::endl;
    ss << "minSpeed     = " << profileCfg.minSpeed     << " steps/s"   << std::endl;
    return ss.str();
}
