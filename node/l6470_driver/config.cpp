//#include "multidriver.h"
#include "types.h"

/////////////////////////
/// Configuration Commands
/////////////////////////

void
MultiDriver::setConfig(const Config &cfg , int motor)
{
    checkMotorIsValid(motor);

    // Set BackEmf Settings
    setBackEmfConfig(cfg.backEmfConfig,motor);

    // Current Thresholds
    setOCThreshold(cfg.overCurrentThreshold , motor);
    setStallThreshold(cfg.stallThreshold , motor);
    setOCShutdown(cfg.overCurrentDetection , motor);

    // Step Mode
    setStepMode(cfg.stepMode , motor);
    setSyncSelect(cfg.syncSelect , cfg.syncEnable , motor);

    // Set Oscillator related configs
    setOscMode(cfg.oscillatorSelect , motor);
    setSwitchMode(cfg.switchConfiguration , motor);
    setSlewRate(cfg.slewRate , motor);
    setVoltageComp(cfg.voltageCompensation , motor);
    setPWMFreq(cfg.pwmFrequencyDivider , cfg.pwmFrequencyMultiplier , motor);
    setAlarmState(cfg.alarmState , motor);
}

Config
MultiDriver::getConfig(int motor)
{
    Config config;

    config.backEmfConfig = getBackEmfConfig(motor);

    // config.thermalDriftCoefficient =

    config.overCurrentThreshold = getOCThreshold(motor);
    config.overCurrentDetection = getOCShutdown(motor);
    config.stallThreshold       = getStallThreshold(motor);

    config.stepMode             = getStepMode(motor);
    config.syncSelect           = getSyncSelect(motor);
    config.syncEnable           = getSyncEnable(motor);

    config.oscillatorSelect         = getOscMode(motor);
    config.switchConfiguration      = getSwitchMode(motor);
    config.slewRate                 = getSlewRate(motor);
    config.voltageCompensation      = getVoltageComp(motor);
    config.pwmFrequencyDivider      = getPWMFreqDivisor(motor);
    config.pwmFrequencyMultiplier   = getPWMFreqMultiplier(motor);
    config.alarmState               = getAlarmState(motor);

    return config;
}


// Setup the SYNC/BUSY pin to be either SYNC or BUSY, and to a desired
//  ticks per step level.
void MultiDriver::setSyncSelect( SyncSelect syncSelect , bool syncEnable , int motor)
{
  // Only some of the bits in this register are of interest to us; we need to
  //  clear those bits. It happens that they are the upper four.
  const uint8_t syncMask = 0x0F;
  uint8_t syncPinConfig = (uint8_t)getParam(STEP_MODE,motor);
  syncPinConfig &= syncMask;

  // Now, let's OR in the arguments. We're going to mask the incoming
  //  data to avoid touching any bits that aren't appropriate. See datasheet
  //  for more info about which bits we're touching.
  const uint8_t syncEnableMask = 0x80;
  const uint8_t syncSelectMask = 0x70;
  syncPinConfig |= ((syncEnable & syncEnableMask) | (syncSelect & syncSelectMask));

  // Now we should be able to send that uint8_t right back to the dSPIN- it
  //  won't corrupt the other bits, and the changes are made.
  setParam(STEP_MODE, (uint32_t)syncPinConfig , motor);
}

void
MultiDriver::setAlarmState(AlarmState alarmState , int motor)
{
    uint8_t alarmStateByte=0x00;
    alarmState.overCurrentEnabled     ? alarmStateByte |= 0x01 : alarmStateByte|=0;
    alarmState.thermalShutdownEnabled ? alarmStateByte |= 0x02 : alarmStateByte|=0;
    alarmState.thermalWarningEnabled  ? alarmStateByte |= 0x04 : alarmStateByte|=0;
    alarmState.underVoltageEnabled    ? alarmStateByte |= 0x08 : alarmStateByte|=0;
    alarmState.stallDetectionAEnabled ? alarmStateByte |= 0x10 : alarmStateByte|=0;
    alarmState.stallDetectionBEnabled ? alarmStateByte |= 0x20 : alarmStateByte|=0;
    alarmState.switchTurnOnEnabled    ? alarmStateByte |= 0x40 : alarmStateByte|=0;
    alarmState.badCommandEnabled      ? alarmStateByte |= 0x80 : alarmStateByte|=0;
    setParam(ALARM_EN,alarmStateByte , motor);
}

AlarmState
MultiDriver::getAlarmState(int motor)
{
    AlarmState alarmState;
    uint8_t alarmStateByte = getParam(ALARM_EN,motor);

    alarmState.overCurrentEnabled     = alarmStateByte & 0x01;
    alarmState.thermalShutdownEnabled = alarmStateByte & 0x02;
    alarmState.thermalWarningEnabled  = alarmStateByte & 0x04;
    alarmState.underVoltageEnabled    = alarmStateByte & 0x08;
    alarmState.stallDetectionAEnabled = alarmStateByte & 0x10;
    alarmState.stallDetectionBEnabled = alarmStateByte & 0x20;
    alarmState.switchTurnOnEnabled    = alarmStateByte & 0x40;
    alarmState.badCommandEnabled      = alarmStateByte & 0x80;

    return alarmState;
}

SyncSelect
MultiDriver::getSyncSelect(int motor)
{
    const uint8_t syncSelectMask = 0x70;
    return static_cast<SyncSelect>(getParam(STEP_MODE,motor) & syncSelectMask);
}

bool
MultiDriver::getSyncEnable(int motor)
{
    const uint8_t syncEnableMask = 0x80;
    return (getParam(STEP_MODE,motor) & syncEnableMask);
}

// The dSPIN chip supports microstepping for a smoother ride. This function
//  provides an easy front end for changing the microstepping mode.
void MultiDriver::setStepMode(StepMode stepMode,int motor)
{
  // Only some of these bits are useful (the lower three). We'll extract the
  //  current contents, clear those three bits, then set them accordingly.
  const uint8_t stepModeMask = 0xF8;
  uint8_t stepModeConfig = (uint8_t)getParam(STEP_MODE,motor);
  stepModeConfig &= stepModeMask;

  // Now we can OR in the new bit settings. Mask the argument so we don't
  //  accidentally the other bits, if the user sends us a non-legit value.
  stepModeConfig |= (stepMode&STEP_MODE_STEP_SEL);

  // Now push the change to the chip.
  setParam(STEP_MODE, (uint32_t)stepModeConfig , motor);
}

StepMode MultiDriver::getStepMode(int motor) {
  return static_cast<StepMode>(getParam(STEP_MODE,motor) & STEP_MODE_STEP_SEL);
}

// Above this threshold, the dSPIN will cease microstepping and go to full-step
//  mode.
void MultiDriver::setFullSpeed(float stepsPerSecond, int motor)
{
  uint32_t integerSpeed = FSCalc(stepsPerSecond);
  setParam(FS_SPD, integerSpeed, motor);
}

float MultiDriver::getFullSpeed(int motor)
{
  return FSParse(getParam(FS_SPD,motor));
}

void MultiDriver::setOCThreshold(CurrentThreshold ocThreshold, int motor)
{
  setParam(OCD_TH, 0x0F & ocThreshold, motor);
}

CurrentThreshold MultiDriver::getOCThreshold(int motor)
{
  return static_cast<CurrentThreshold> (getParam(OCD_TH, motor) & CONFIG_OC_THRESOLD_REG);
}

void
MultiDriver::setStallThreshold(CurrentThreshold stallCurrent, int motor)
{
    setParam(STALL_TH,0x0F & stallCurrent, motor);
}

CurrentThreshold
MultiDriver::getStallThreshold(int motor)
{
    const uint32_t stallThresholdMask=0xFF; // TODO - fix!!
    return static_cast<CurrentThreshold> (getParam(STALL_TH, motor) & stallThresholdMask);
}

// The next few functions are all breakouts for individual options within the
//  single register CONFIG. We'll read CONFIG, blank some bits, then OR in the
//  new value.

// This is a multiplier/divider setup for the PWM frequency when microstepping.
//  Divisors of 1-7 are available; multipliers of .625-2 are available. See
//  datasheet for more details; it's not clear what the frequency being
//  multiplied/divided here is, but it is clearly *not* the actual clock freq.
void MultiDriver::setPWMFreq(PwmFrequencyDivider divider, PwmFrequencyMultiplier multiplier, int motor)
{
  uint32_t configVal = getParam(CONFIG, motor);

  // The divisor is set by config 15:13, so mask 0xE000 to clear them.
  configVal &= ~(CONFIG_F_PWM_INT);
  // The multiplier is set by config 12:10; mask is 0x1C00
  configVal &= ~(CONFIG_F_PWM_DEC);
  // Now we can OR in the masked-out versions of the values passed in.
  configVal |= ((CONFIG_F_PWM_INT&divider)|(CONFIG_F_PWM_DEC&multiplier));
  setParam(CONFIG, configVal , motor);
}

PwmFrequencyDivider MultiDriver::getPWMFreqDivisor(int motor)
{
  return static_cast<PwmFrequencyDivider> (getParam(CONFIG, motor) & CONFIG_F_PWM_INT);
}

PwmFrequencyMultiplier MultiDriver::getPWMFreqMultiplier(int motor)
{
  return static_cast<PwmFrequencyMultiplier> (getParam(CONFIG, motor) & CONFIG_F_PWM_DEC);
}

// Slew rate of the output in V/us. Can be 180, 290, or 530.
void MultiDriver::setSlewRate(SlewRate slewRate, int motor)
{
  uint32_t configVal = getParam(CONFIG, motor);

  // These bits live in CONFIG 9:8, so the mask is 0x0300.
  configVal &= ~(CONFIG_SLEW_RATE_MASK);
  //Now, OR in the masked incoming value.
  configVal |= (CONFIG_SLEW_RATE_MASK&slewRate);
  setParam(CONFIG, configVal, motor);
}

SlewRate MultiDriver::getSlewRate(int motor)
{
  return static_cast<SlewRate> (getParam(CONFIG, motor) & CONFIG_SLEW_RATE_MASK);
}

// Single bit- do we shutdown the drivers on overcurrent or not?
void MultiDriver::setOCShutdown(OverCurrentDetection overCurrentDetection, int motor)
{
  uint32_t configVal = getParam(CONFIG, motor);
  // This bit is CONFIG 7, mask is 0x0080
  configVal &= ~(CONFIG_OC_DETECTION_MASK);
  //Now, OR in the masked incoming value.
  configVal |= (CONFIG_OC_DETECTION_MASK&overCurrentDetection);
  setParam(CONFIG, configVal, motor);
}

OverCurrentDetection MultiDriver::getOCShutdown(int motor)
{
  return static_cast<OverCurrentDetection> (getParam(CONFIG, motor) & CONFIG_OC_DETECTION_MASK);
}

// Enable motor voltage compensation? Not at all straightforward- check out
//  p34 of the datasheet.
void MultiDriver::setVoltageComp(VoltageCompensation vsCompMode, int motor)
{
  uint32_t configVal = getParam(CONFIG, motor);
  // This bit is CONFIG 5, mask is 0x0020
  configVal &= ~(CONFIG_EN_VSCOMP_MASK);
  //Now, OR in the masked incoming value.
  configVal |= (CONFIG_EN_VSCOMP_MASK&vsCompMode);
  setParam(CONFIG, configVal, motor);
}

VoltageCompensation MultiDriver::getVoltageComp(int motor)
{
  return static_cast <VoltageCompensation> (getParam(CONFIG,motor) & CONFIG_EN_VSCOMP_MASK);
}

// The switch input can either hard-stop the driver _or_ activate an interrupt.
//  This bit allows you to select what it does.
void MultiDriver::setSwitchMode(SwitchConfiguration switchMode, int motor)
{
  uint32_t configVal = getParam(CONFIG, motor);
  // This bit is CONFIG 4, mask is 0x0010
  configVal &= ~(CONFIG_SW_MODE_MASK);
  //Now, OR in the masked incoming value.
  configVal |= (CONFIG_SW_MODE_MASK & switchMode);
  setParam(CONFIG, configVal, motor);
}

SwitchConfiguration MultiDriver::getSwitchMode(int motor)
{
  return static_cast <SwitchConfiguration> (getParam(CONFIG, motor) & CONFIG_SW_MODE_MASK);
}

// There are a number of clock options for this chip- it can be configured to
//  accept a clock, drive a crystal or resonator, and pass or not pass the
//  clock signal downstream. Theoretically, you can use pretty much any
//  frequency you want to drive it; practically, this library assumes it's
//  being driven at 16MHz. Also, the device will use these bits to set the
//  math used to figure out steps per second and stuff like that.
void MultiDriver::setOscMode(OscillatorSelect oscillatorMode, int motor)
{
  uint32_t configVal = getParam(CONFIG , motor);
  // These bits are CONFIG 3:0, mask is 0x000F
  configVal &= ~(CONFIG_OSC_SEL_MASK);
  //Now, OR in the masked incoming value.
  configVal |= (CONFIG_OSC_SEL_MASK&oscillatorMode);
  setParam(CONFIG, configVal, motor);
}

OscillatorSelect MultiDriver::getOscMode(int motor)
{
  return static_cast <OscillatorSelect> (getParam(CONFIG,motor) & CONFIG_OSC_SEL_MASK);
}

// The KVAL registers are...weird. I don't entirely understand how they differ
//  from the microstepping, but if you have trouble getting the motor to run,
//  tweaking KVAL has proven effective in the past. There's a separate register
//  for each case: running, static, accelerating, and decelerating.

void MultiDriver::setAccKVAL(uint8_t kvalInput, int motor)
{
  setParam(KVAL_ACC, kvalInput, motor);
}

uint8_t MultiDriver::getAccKVAL(int motor)
{
  return (uint8_t) getParam(KVAL_ACC, motor);
}

void MultiDriver::setDecKVAL(uint8_t kvalInput, int motor)
{
  setParam(KVAL_DEC, kvalInput, motor);
}

uint8_t MultiDriver::getDecKVAL(int motor)
{
  return (uint8_t) getParam(KVAL_DEC, motor);
}

void MultiDriver::setRunKVAL(uint8_t kvalInput, int motor)
{
  setParam(KVAL_RUN, kvalInput, motor);
}

uint8_t MultiDriver::getRunKVAL(int motor)
{
  return (uint8_t) getParam(KVAL_RUN, motor);
}

void MultiDriver::setHoldKVAL(uint8_t kvalInput, int motor)
{
  setParam(KVAL_HOLD, kvalInput, motor);
}

uint8_t MultiDriver::getHoldKVAL(int motor)
{
  return (uint8_t) getParam(KVAL_HOLD,motor);
}

void
MultiDriver::setBackEmfConfig(const VoltageModeCfg &backEmfConfig, int motor)
{
    // Set the K Values
    setHoldKVAL(backEmfConfig.holdingKVal,motor);
    setRunKVAL(backEmfConfig.constantSpeedKVal,motor);
    setAccKVAL(backEmfConfig.accelStartingKVal,motor);
    setDecKVAL(backEmfConfig.decelStartingKVal,motor);

    // Set the intersect speed and slope of the curve
    setParam(INT_SPD,backEmfConfig.intersectSpeed,motor);
    setParam(ST_SLP,backEmfConfig.startSlope,motor);
    setParam(FN_SLP_ACC,backEmfConfig.accelFinalSlope,motor);
    setParam(FN_SLP_DEC,backEmfConfig.decelFinalSlope,motor);
}

VoltageModeCfg
MultiDriver::getBackEmfConfig(int motor)
{
    VoltageModeCfg backEmfConfig;

    // Get the K Values
    backEmfConfig.holdingKVal = getHoldKVAL(motor);
    backEmfConfig.constantSpeedKVal = getRunKVAL(motor);
    backEmfConfig.accelStartingKVal = getAccKVAL(motor);
    backEmfConfig.decelStartingKVal = getDecKVAL(motor);

    // Get the intersect speed and slope of the curve
    backEmfConfig.intersectSpeed  = getParam(INT_SPD, motor);
    backEmfConfig.startSlope      = getParam(ST_SLP, motor);
    backEmfConfig.accelFinalSlope = getParam(FN_SLP_ACC, motor);
    backEmfConfig.decelFinalSlope = getParam(FN_SLP_DEC, motor);

    return backEmfConfig;
}

// Enable or disable the low-speed optimization option. With LSPD_OPT enabled,
//  motion starts from 0 instead of MIN_SPEED and low-speed optimization keeps
//  the driving sine wave prettier than normal until MIN_SPEED is reached.
void MultiDriver::setLoSpdOpt(bool enable, int motor)
{
  uint32_t temp = getParam(MIN_SPEED, motor);
  if (enable) temp |= 0x00001000; // Set the LSPD_OPT bit
  else        temp &= 0xffffefff; // Clear the LSPD_OPT bit
  setParam(MIN_SPEED, temp, motor);
}

bool MultiDriver::getLoSpdOpt(int motor)
{
  return (bool) ((getParam(MIN_SPEED,motor) & 0x00001000) != 0);
}
