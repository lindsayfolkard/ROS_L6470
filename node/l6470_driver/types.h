#pragma once

#include <iostream>
#include <map>
#include <boost/bimap.hpp>
#include <boost/optional.hpp>

std::string toMapString(const std::map <int,uint32_t> &values , uint8_t bitLength);
std::string toLineString(uint8_t *buffer , uint8_t length);
std::string toLineString(const std::vector<uint8_t> &data);

template <typename L, typename R>
boost::bimap<L, R>
makeBiMap(std::initializer_list<typename boost::bimap<L, R>::value_type> list)
{
    return boost::bimap<L, R>(list.begin(), list.end());
}

// constant definitions for overcurrent thresholds. Write these values to 
//  register OCD_TH to set the level at which an overcurrent even occurs.
#define CONFIG_OC_THRESOLD_REG 0XF
enum CurrentThreshold
{
    OCD_TH_375m =  0x00,
    OCD_TH_750m =  0x01,
    OCD_TH_1125m = 0x02,
    OCD_TH_1500m = 0x03,
    OCD_TH_1875m = 0x04,
    OCD_TH_2250m = 0x05,
    OCD_TH_2625m = 0x06,
    OCD_TH_3000m = 0x07,
    OCD_TH_3375m = 0x08,
    OCD_TH_3750m = 0x09,
    OCD_TH_4125m = 0x0A,
    OCD_TH_4500m = 0x0B,
    OCD_TH_4875m = 0x0C,
    OCD_TH_5250m = 0x0D,
    OCD_TH_5625m = 0x0E,
    OCD_TH_6000m = 0x0F,
    OCD_TH_6375m = 0x10,
    OCD_TH_6750m = 0x11,
    OCD_TH_7125m = 0x12,
    OCD_TH_7500m = 0x13,
    OCD_TH_7875m = 0x14,
    OCD_TH_8250m = 0x15,
    OCD_TH_8625m = 0x16,
    OCD_TH_9000m = 0x17,
    OCD_TH_9375m = 0x18,
    OCD_TH_9750m = 0x19,
    OCD_TH_10125m = 0x1A,
    OCD_TH_10500m = 0x1B,
    OCD_TH_10875m = 0x1C,
    OCD_TH_11250m = 0x1D,
    OCD_TH_11625m = 0x1E,
    OCD_TH_12000m = 0x1F,
//    OCD_TH_12375m = 0x21,
//    OCD_TH_12750m = 0x22,
//    OCD_TH_13125m = 0x23,
//    OCD_TH_13500m = 0x24,
//    OCD_TH_13875m = 0x25
};

boost::bimap <CurrentThreshold,std::string> getCurrentThresholdBiMap();
std::string toString(CurrentThreshold currentThreshold);
inline std::ostream& operator<<(std::ostream& os, CurrentThreshold currentThreshold)
{
    return os << toString(currentThreshold);
}

enum ThermalDriftCompensation
{
    ThermalDrift_1 = 0,
    ThermalDrift_1_03125 = 1,
    ThermalDrift_1_0625 = 2,
    ThermalDrift_1_09375 = 3,
    ThermalDrift_1_125 = 4,
    ThermalDrift_1_15625 = 5,
    ThermalDrift_1_1875 = 6,
    ThermalDrift_1_21875 = 7,
    ThermalDrift_1_25 = 8,
    ThermalDrift_1_28125 = 9,
    ThermalDrift_1_3125 = 10,
    ThermalDrift_1_34375 = 11,
    ThermalDrift_1_375 = 12,
    ThermalDrift_1_40625 = 13,
    ThermalDrift_1_4375 = 14,
    ThermalDrift_1_46875 = 15
};

boost::bimap<ThermalDriftCompensation,std::string> getThermalDriftCompensationBiMap();
std::string toString(ThermalDriftCompensation thermalDriftCompensation);
inline std::ostream& operator<<(std::ostream& os, ThermalDriftCompensation x)
{
    return os << toString(x);
}

// STEP_MODE option values.
// First comes the "microsteps per step" options...
#define STEP_MODE_STEP_SEL 0x07  // Mask for these bits only.

enum StepMode
{
    STEP_SEL_1      = 0x00,
    STEP_SEL_1_2    = 0x01,
    STEP_SEL_1_4    = 0x02,
    STEP_SEL_1_8    = 0x03,
    STEP_SEL_1_16   = 0x04,
    STEP_SEL_1_32   = 0x05,
    STEP_SEL_1_64   = 0x06,
    STEP_SEL_1_128  = 0x07
};

boost::bimap<StepMode,std::string> getStepModeBiMap();
std::string toString(StepMode stepMode);
inline std::ostream& operator<<(std::ostream& os, StepMode stepMode)
{
    return os << toString(stepMode);
}

// ...next, define the SYNC_EN bit. When set, the BUSYN pin will instead
//  output a clock related to the full-step frequency as defined by the
//  SYNC_SEL bits below.
#define STEP_MODE_SYNC_EN	 0x80  // Mask for this bit
//#define SYNC_EN 0x80

// ...last, define the SYNC_SEL modes. The clock output is defined by
//  the full-step frequency and the value in these bits- see the datasheet
//  for a matrix describing that relationship (page 46).
#define STEP_MODE_SYNC_SEL_ 0x70

enum SyncSelect
{
    SYNC_SEL_1_2 = 0x00,
    SYNC_SEL_1   = 0x10,
    SYNC_SEL_2   = 0x20,
    SYNC_SEL_4   = 0x30,
    SYNC_SEL_8   = 0x40,
    SYNC_SEL_16  = 0x50,
    SYNC_SEL_32  = 0x60,
    SYNC_SEL_64  = 0x70
};
boost::bimap<SyncSelect,std::string> getSyncSelectBiMap();
std::string toString(SyncSelect syncSelect);
inline std::ostream& operator<<(std::ostream& os,SyncSelect syncSelect)
{
    return os << toString(syncSelect);
}

enum ControlMode
{
    VoltageControlMode = 0x00,
    CurrentControlMode = 0x01
};
boost::bimap<ControlMode,std::string> getControlModeBiMap();
std::string toString(ControlMode controlMode);
inline std::ostream& operator<<(std::ostream &os, ControlMode controlMode)
{
    return os << toString(controlMode);
}

// Bit names for the ALARM_EN register.
//  Each of these bits defines one potential alarm condition.
//  When one of these conditions occurs and the respective bit in ALARM_EN is set,
//  the FLAG pin will go low. The register must be queried to determine which event
//  caused the alarm.

#define ALARM_EN_OVERCURRENT       0x01
#define ALARM_EN_THERMAL_SHUTDOWN  0x02
#define ALARM_EN_THERMAL_WARNING   0x04
#define ALARM_EN_UNDER_VOLTAGE     0x08
#define ALARM_EN_STALL_DET_A       0x10
#define ALARM_EN_STALL_DET_B       0x20
#define ALARM_EN_SW_TURN_ON        0x40
#define ALARM_EN_WRONG_NPERF_CMD   0x80

struct AlarmState
{
    AlarmState();

    bool overCurrentEnabled;
    bool thermalShutdownEnabled;
    bool thermalWarningEnabled;
    bool underVoltageEnabled;
    bool stallDetectionAEnabled;
    bool stallDetectionBEnabled;
    bool switchTurnOnEnabled;
    bool badCommandEnabled;
};
std::string toString(AlarmState alarmState);
inline std::ostream& operator<<(std::ostream& os,AlarmState alarmState)
{
    return os << toString(alarmState);
}

// CONFIG register renames.

// Oscillator options.
// The dSPIN needs to know what the clock frequency is because it uses that for some
//  calculations during operation.
#define CONFIG_OSC_SEL_MASK             0x000F // Mask for this bit field.

enum OscillatorSelect
{
    CONFIG_INT_16MHZ                = 0x0000 , // Internal 16MHz, no output
    CONFIG_INT_16MHZ_OSCOUT_2MHZ    = 0x0008 , // Default; internal 16MHz, 2MHz output
    CONFIG_INT_16MHZ_OSCOUT_4MHZ    = 0x0009 , // Internal 16MHz, 4MHz output
    CONFIG_INT_16MHZ_OSCOUT_8MHZ    = 0x000A , // Internal 16MHz, 8MHz output
    CONFIG_INT_16MHZ_OSCOUT_16MHZ   = 0x000B , // Internal 16MHz, 16MHz output
    CONFIG_EXT_8MHZ_XTAL_DRIVE      = 0x0004 , // External 8MHz crystal
    CONFIG_EXT_16MHZ_XTAL_DRIVE     = 0x0005 , // External 16MHz crystal
    CONFIG_EXT_24MHZ_XTAL_DRIVE     = 0x0006 , // External 24MHz crystal
    CONFIG_EXT_32MHZ_XTAL_DRIVE     = 0x0007 , // External 32MHz crystal
    CONFIG_EXT_8MHZ_OSCOUT_INVERT   = 0x000C , // External 8MHz crystal, output inverted
    CONFIG_EXT_16MHZ_OSCOUT_INVERT  = 0x000D , // External 16MHz crystal, output inverted
    CONFIG_EXT_24MHZ_OSCOUT_INVERT  = 0x000E , // External 24MHz crystal, output inverted
    CONFIG_EXT_32MHZ_OSCOUT_INVERT  = 0x000F   // External 32MHz crystal, output inverted
};

boost::bimap<OscillatorSelect,std::string> getOscillatorSelectBiMap();
std::string toString(OscillatorSelect oscillatorSelect);
inline std::ostream& operator<<(std::ostream& os,OscillatorSelect oscillatorSelect)
{
    return os << toString(oscillatorSelect);
}

// Configure the functionality of the external switch input
#define CONFIG_SW_MODE_MASK 0x0010 // Mask for this bit.
enum SwitchConfiguration
{
    CONFIG_SW_HARD_STOP = 0x0000, // Default; hard stop motor on switch.
    CONFIG_SW_USER      = 0x0010  // Tie to the GoUntil and ReleaseSW

    //  commands to provide jog function.
    //  See page 25 of datasheet.
};

boost::bimap<SwitchConfiguration,std::string> getSwitchConfigurationBiMap();
std::string toString(SwitchConfiguration switchConfiguration);
inline std::ostream& operator<<(std::ostream& os,SwitchConfiguration switchConfiguration)
{
    return os << toString(switchConfiguration);
}

// Configure the motor voltage compensation mode (see page 34 of datasheet)
#define CONFIG_EN_VSCOMP_MASK               0x0020  // Mask for this bit.
enum VoltageCompensation
{
    CONFIG_VS_COMP_DISABLE = 0x0000,  // Disable motor voltage compensation.
    CONFIG_VS_COMP_ENABLE  = 0x0020   // Enable motor voltage compensation.
};

boost::bimap<VoltageCompensation,std::string> getVoltageCompensationBiMap();
std::string toString(VoltageCompensation voltageCompensation);
inline std::ostream& operator<<(std::ostream& os,VoltageCompensation voltageCompensation)
{
    return os << toString(voltageCompensation);
}

// Configure overcurrent detection event handling
#define CONFIG_OC_DETECTION_MASK                   0x0080  // Mask for this bit.
enum OverCurrentDetection
{
    CONFIG_OC_SD_DISABLE = 0x0000,   // Bridges do NOT shutdown on OC detect
    CONFIG_OC_SD_ENABLE  = 0x0080    // Bridges shutdown on OC detect
};

boost::bimap<OverCurrentDetection,std::string> getOverCurrentDetectionBiMap();
std::string toString(OverCurrentDetection overCurrentDetection);
inline std::ostream& operator<<(std::ostream& os,OverCurrentDetection x)
{
    return os << toString(x);
}

// Configure the slew rate of the power bridge output
#define CONFIG_SLEW_RATE_MASK                  0x0300  // Mask for this bit field.
enum SlewRate
{
    CONFIG_SR_180V_us = 0x0000, // 180V/us
    CONFIG_SR_290V_us = 0x0200, // 290V/us
    CONFIG_SR_530V_us = 0x0300  // 530V/us
};

boost::bimap<SlewRate,std::string> getSlewRateBiMap();
std::string toString(SlewRate slewRate);
inline std::ostream& operator<<(std::ostream& os,SlewRate x)
{
    return os << toString(x);
}

// Integer divisors for PWM sinewave generation
//  See page 32 of the datasheet for more information on this.
#define CONFIG_F_PWM_DEC               0x1C00      // mask for this bit field
enum PwmFrequencyMultiplier
{
    CONFIG_PWM_MUL_0_625  = (0x00)<<10,
    CONFIG_PWM_MUL_0_75   = (0x01)<<10,
    CONFIG_PWM_MUL_0_875  = (0x02)<<10,
    CONFIG_PWM_MUL_1      = (0x03)<<10,
    CONFIG_PWM_MUL_1_25   = (0x04)<<10,
    CONFIG_PWM_MUL_1_5    = (0x05)<<10,
    CONFIG_PWM_MUL_1_75   = (0x06)<<10,
    CONFIG_PWM_MUL_2      = (0x07)<<10
};

boost::bimap<PwmFrequencyMultiplier,std::string> getPwmFrequencyMultiplierBiMap();
std::string toString(PwmFrequencyMultiplier pwmFrequencyMultiplier);
inline std::ostream& operator<<(std::ostream& os,PwmFrequencyMultiplier x)
{
    return os << toString(x);
}

// Multiplier for the PWM sinewave frequency
#define CONFIG_F_PWM_INT               0xE000     // mask for this bit field.
enum PwmFrequencyDivider
{
    CONFIG_PWM_DIV_1 = (0x00)<<13,
    CONFIG_PWM_DIV_2 = (0x01)<<13,
    CONFIG_PWM_DIV_3 = (0x02)<<13,
    CONFIG_PWM_DIV_4 = (0x03)<<13,
    CONFIG_PWM_DIV_5 = (0x04)<<13,
    CONFIG_PWM_DIV_6 = (0x05)<<13,
    CONFIG_PWM_DIV_7 = (0x06)<<13
};

boost::bimap<PwmFrequencyDivider,std::string> getPwmFrequencyDividerBiMap();
std::string toString (PwmFrequencyDivider pwmFrequency);
inline std::ostream& operator<<(std::ostream& os,PwmFrequencyDivider x)
{
    return os << toString(x);
}

enum MotorSpinDirection
{
    Reverse=0x00,
    Forward=0x01
};
std::string toString (MotorSpinDirection motorSpinDirection);
inline std::ostream& operator<<(std::ostream& os,MotorSpinDirection x)
{
    return os << toString(x);
}

// Status register motor status field
// field mask
enum MotorStatus
{
    STATUS_MOT_STATUS_STOPPED        = (0x0000)<<13, // Motor stopped
    STATUS_MOT_STATUS_ACCELERATION   = (0x0001)<<13,// Motor accelerating
    STATUS_MOT_STATUS_DECELERATION   = (0x0002)<<13, // Motor decelerating
    STATUS_MOT_STATUS_CONST_SPD      = (0x0003)<<13 // Motor at constant speed
};
std::string toString (MotorStatus motorStatus);
inline std::ostream& operator<<(std::ostream& os,MotorStatus x)
{
    return os << toString(x);
}

// Status register bit renames- read-only bits conferring information about the
//  device to the user.
#define STATUS_HIZ                     0x0001 // high when bridges are in HiZ mode
#define STATUS_BUSY                    0x0002 // mirrors BUSY pin
#define STATUS_SW_F                    0x0004 // low when switch open, high when closed
#define STATUS_SW_EVN                  0x0008 // active high, set on switch falling edge,
#define STATUS_DIR                     0x0010 // Indicates current motor direction.
#define STATUS_MOT_STATUS              0x0060

#define STATUS_NOTPERF_CMD             0x0080 // Last command not performed.
#define STATUS_SCK_MOD                 0x0100 // Step clock mode is active

#define STATUS_UVLO                    0x0200 // Undervoltage lockout is active
#define STATUS_UVLO_ADC                0x0400 // Undervotlage adc event
#define STATUS_TH_WRN                  0x0800 // Thermal warning
#define STATUS_TH_SD                   0x1000 // Thermal shutdown
#define STATUS_OCD                     0x2000 // Overcurrent detected
#define STATUS_STEP_LOSS_A             0x4000 // Stall detected on A bridge
#define STATUS_STEP_LOSS_B             0x8000 // Stall detected on B bridge

#define DIR_SHIFT 4
#define MOT_STATUS_SHIFT 5
// Easier higher up to deal with a status struct of bools as opposed to bit&
struct Status
{
    // General ControllerState
    bool isHighZ;
    bool isBusy;
    bool isSwitchClosed;
    bool switchEventDetected;
    bool stepClockActive;

    // Fault States
    bool commandError;

    bool underVoltageLockout;
    bool underVoltageAdcLockout;

    bool hasThermalWarning;
    bool isInThermalShutdown;

    bool overCurrentDetected;

    bool stallDetectedPhaseA;
    bool stallDetectedPhaseB;

    // Motor State
    MotorSpinDirection spinDirection;
    MotorStatus        motorStatus;

    // Speed and Position
    int32_t  position; // steps from home
    uint32_t speed; // steps/s
};
std::string toString (Status &status);
inline std::ostream& operator<<(std::ostream& os,Status x)
{
    return os << toString(x);
}
Status parseStatus (uint16_t statusValue);

///
/// Gate Configuration Registers (PowerStep01 only)
///

// The IGATE parameter selects the sink/source current used by gate driving circuitry to
// charge/discharge the respective gate during commutations. S
enum GateCurrent
{
    GateCurrent_2ma  = 0x00,
    GateCurrent_4ma  = 0x01,
    GateCurrent_8ma  = 0x02,
    GateCurrent_16ma = 0x03,
    GateCurrent_24ma = 0x04,
    GateCurrent_32ma = 0x05,
    GateCurrent_64ma = 0x06,
    GateCurrent_96ma = 0x07
};

boost::bimap<GateCurrent,std::string> getGateCurrentBiMap();
std::string toString(GateCurrent x);
inline std::ostream& operator<<(std::ostream& os, GateCurrent x)
{
    return os << toString(x);
}

// The TCC parameter defines the duration of constant current phase
// during gate turn-on and turn-off sequences
enum GateTcc
{
    GateTcc_125ns  = 0x00,
    GateTcc_250ns  = 0x01,
    GateTcc_375ns  = 0x02,
    GateTcc_500ns  = 0x03,
    GateTcc_625ns  = 0x04,
    GateTcc_750ns  = 0x05,
    GateTcc_875ns  = 0x06,
    GateTcc_1000ns = 0x07,
    GateTcc_1125ns = 0x08,
    GateTcc_1250ns = 0x09,
    GateTcc_1375ns = 0x0A,
    GateTcc_1500ns = 0x0B,
    GateTcc_1625ns = 0x0C,
    GateTcc_1750ns = 0x0D,
    GateTcc_1875ns = 0x0E,
    GateTcc_2000ns = 0x0F,
    GateTcc_2125ns = 0x10,
    GateTcc_2250ns = 0x11,
    GateTcc_2375ns = 0x12,
    GateTcc_2500ns = 0x13,
    GateTcc_2625ns = 0x14,
    GateTcc_2750ns = 0x15,
    GateTcc_2875ns = 0x16,
    GateTcc_3000ns = 0x17,
    GateTcc_3125ns = 0x18,
    GateTcc_3250ns = 0x19,
    GateTcc_3375ns = 0x1A,
    GateTcc_3500ns = 0x1B,
    GateTcc_3625ns = 0x1C,
    GateTcc_3750ns = 0x1D,
    GateTcc_3875ns = 0x1E,
    GateTcc_4000ns = 0x1F,
};

boost::bimap<GateTcc,std::string> getGateTccBiMap();
std::string toString(GateTcc x);
inline std::ostream& operator<<(std::ostream& os, GateTcc x)
{
    return os << toString(x);
}

enum GateTBoost
{
    GateTBoost_0ns    = 0x00,
    GateTBoost_62_5ns = 0x01, // different with clock frequency != 16MHZ
    GateTBoost_125ns  = 0x02,
    GateTBoost_250ns  = 0x03,
    GateTBoost_375ns  = 0x04,
    GateTBoost_500ns  = 0x05,
    GateTBoost_750ns  = 0x06,
    GateTBoost_1000ns = 0x07
};

boost::bimap<GateTBoost,std::string> getGateTBoostBiMap();
std::string toString(GateTBoost x);
inline std::ostream& operator<<(std::ostream& os, GateTBoost x)
{
    return os << toString(x);
}

struct GateConfig1
{
    // TODO - get the default values that are likely to be best (either from doc or controller itself)
    GateCurrent gateCurrent = GateCurrent_16ma;
    GateTcc     gateTcc     = GateTcc_2000ns;
    GateTBoost  gateTBoost  = GateTBoost_375ns ;
    bool        wd_en       = false;
};

std::string toString(GateConfig1 x);
inline std::ostream& operator<<(std::ostream& os, const GateConfig1 &x)
{
    return os << toString(x);
}

///
/// GateConfig2 Settings
///

enum GateDeadTime
{
    GateDeadTime_125ns  = 0x00,
    GateDeadTime_250ns  = 0x01,
    GateDeadTime_375ns  = 0x02,
    GateDeadTime_500ns  = 0x03,
    GateDeadTime_625ns  = 0x04,
    GateDeadTime_750ns  = 0x05,
    GateDeadTime_875ns  = 0x06,
    GateDeadTime_1000ns = 0x07,
    GateDeadTime_1125ns = 0x08,
    GateDeadTime_1250ns = 0x09,
    GateDeadTime_1375ns = 0x0A,
    GateDeadTime_1500ns = 0x0B,
    GateDeadTime_1625ns = 0x0C,
    GateDeadTime_1750ns = 0x0D,
    GateDeadTime_1875ns = 0x0E,
    GateDeadTime_2000ns = 0x0F,
    GateDeadTime_2125ns = 0x10,
    GateDeadTime_2250ns = 0x11,
    GateDeadTime_2375ns = 0x12,
    GateDeadTime_2500ns = 0x13,
    GateDeadTime_2625ns = 0x14,
    GateDeadTime_2750ns = 0x15,
    GateDeadTime_2875ns = 0x16,
    GateDeadTime_3000ns = 0x17,
    GateDeadTime_3125ns = 0x18,
    GateDeadTime_3250ns = 0x19,
    GateDeadTime_3375ns = 0x1A,
    GateDeadTime_3500ns = 0x1B,
    GateDeadTime_3625ns = 0x1C,
    GateDeadTime_3750ns = 0x1D,
    GateDeadTime_3875ns = 0x1E,
    GateDeadTime_4000ns = 0x1F
};

boost::bimap<GateDeadTime,std::string> getGateDeadTimeBiMap();
std::string toString(GateDeadTime x);
inline std::ostream& operator<<(std::ostream& os, GateDeadTime x)
{
    return os << toString(x);
}

enum GateTBlank
{
    GateTBlank_125ns  = 0x00,
    GateTBlank_250ns  = 0x01,
    GateTBlank_375ns  = 0x02,
    GateTBlank_500ns  = 0x03,
    GateTBlank_625ns  = 0x04,
    GateTBlank_750ns  = 0x05,
    GateTBlank_875ns  = 0x06,
    GateTBlank_1000ns = 0x07
};

boost::bimap<GateTBlank,std::string> getGateTBlankBiMap();
std::string toString(GateTBlank x);
inline std::ostream& operator<<(std::ostream& os, GateTBlank x)
{
    return os << toString(x);
}

struct GateConfig2
{
    // #TODO - get some good defaults
    GateDeadTime gateDeadTime = GateDeadTime_2000ns;
    GateTBlank   gateTBlank   = GateTBlank_500ns;
};

std::string toString(GateConfig2 x);
inline std::ostream& operator<<(std::ostream& os, GateConfig2 x)
{
    return os << toString(x);
}

// Register address redefines.
//  See the Param_Handler() function for more info about these.
enum ParamRegister
{
    ABS_POS               = 0x01,
    EL_POS                = 0x02,
    MARK                  = 0x03,
    SPEED                 = 0x04,
    ACC                   = 0x05,
    DECEL                 = 0x06,
    MAX_SPEED             = 0x07,
    MIN_SPEED             = 0x08,
    KVAL_HOLD             = 0x09,
    KVAL_RUN              = 0x0A,
    KVAL_ACC              = 0x0B,
    KVAL_DEC              = 0x0C,
    INT_SPD               = 0x0D,
    ST_SLP                = 0x0E,
    FN_SLP_ACC            = 0x0F,
    FN_SLP_DEC            = 0x10,
    K_THERM               = 0x11,
    ADC_OUT               = 0x12,
    OCD_TH                = 0x13,
    STALL_TH              = 0x14,
    FS_SPD                = 0x15,
    STEP_MODE             = 0x16,
    ALARM_EN              = 0x17,
    GATE_CFG1             = 0x18,
    GATE_CFG2             = 0x19,
    CONFIG                = 0x1A, // different on l6470
    STATUS                = 0x1B  // different on l6470
};
std::string toString(ParamRegister paramRegister);
inline std::ostream& operator<<(std::ostream& os,ParamRegister x)
{
    return os << toString(x);
}

//dSPIN commands
enum Command
{
    NOP                   = 0x00,
    SET_PARAM             = 0x00,
    GET_PARAM             = 0x20,
    RUN                   = 0x50,
    STEP_CLOCK            = 0x58,
    MOVE                  = 0x40,
    GOTO                  = 0x60,
    GOTO_DIR              = 0x68,
    GO_UNTIL              = 0x82,
    RELEASE_SW            = 0x92,
    GO_HOME               = 0x70,
    GO_MARK               = 0x78,
    RESET_POS             = 0xD8,
    RESET_DEVICE          = 0xC0,
    SOFT_STOP             = 0xB0,
    HARD_STOP             = 0xB8,
    SOFT_HIZ              = 0xA0,
    HARD_HIZ              = 0xA8,
    GET_STATUS            = 0xD0
};
std::string toString(Command command);
inline std::ostream& operator<<(std::ostream& os,Command x)
{
    return os << toString(x);
}

enum TargetSwitchingPeriod
{
    SwitchingPeriod250KHz  = 0x00,
    SwitchingPeriod125KHz  = 0x01,
    SwitchingPeriod62_5KHz = 0x02,
    SwitchingPeriod31KHz   = 0x03,
    SwitchingPeriod32_     = 0x04,
    SwitchingPeriod16KHz   = 0x0E,
    SwitchingPeriod8KHz    = 0x0F
};

// It is expected during normal operation that these configs will be
// changed several times on the fly
struct ProfileCfg
{
    float acceleration; // steps/s^2
    float deceleration; // steps/s^2
    float maxSpeed; // steps/s
    float minSpeed; // steps/s
};
std::string toString(const ProfileCfg &profileCfg);
inline std::ostream& operator<<(std::ostream& os,const ProfileCfg &x)
{
    return os << toString(x);
}
