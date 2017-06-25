#include "driver.h"

void
AutoDriver::setConfig(const Config &cfg)
{
    // Set BackEmf Settings
    setBackEmfConfig(cfg.backEmfConfig);

    // Motor Thermal Drift (ignore for now)
    // setTHermalDrift

    // Current Thresholds
    setOCThreshold(cfg.overCurrentThreshold);
    setStallThreshold(cfg.stallThreshold);
    setOCShutdown(cfg.overCurrentDetection);

    // Step Mode
    setStepMode(cfg.stepMode);
    setSyncSelect(cfg.syncSelect,cfg.syncEnable);

    // Set Oscillator related configs
    setOscMode(cfg.oscillatorSelect);
    setSwitchMode(cfg.switchConfiguration);
    setSlewRate(cfg.slewRate);
    setVoltageComp(cfg.voltageCompensation);
    setPWMFreq(cfg.pwmFrequencyDivider,cfg.pwmFrequencyMultiplier);
    setAlarmState(cfg.alarmState);

}

Config
AutoDriver::getConfig()
{
    Config config;

    config.backEmfConfig = getBackEmfConfig();

    // config.thermalDriftCoefficient =

    config.overCurrentThreshold = getOCThreshold();
    config.overCurrentDetection = getOCShutdown();
    config.stallThreshold       = getStallThreshold();

    config.stepMode             = getStepMode();
    config.syncSelect           = getSyncSelect();
    config.syncEnable           = getSyncEnable();

    config.oscillatorSelect         = getOscMode();
    config.switchConfiguration      = getSwitchMode();
    config.slewRate                 = getSlewRate();
    config.voltageCompensation      = getVoltageComp();
    config.pwmFrequencyDivider      = getPWMFreqDivisor();
    config.pwmFrequencyMultiplier   = getPWMFreqMultiplier();
    config.alarmState               = getAlarmState();

    return config;
}

void
AutoDriver::setProfileCfg(const ProfileCfg &cfg)
{
    setAcc(cfg.acceleration);
    setDec(cfg.deceleration);
    setMaxSpeed(cfg.maxSpeed);
    setMinSpeed(cfg.minSpeed);
}

ProfileCfg
AutoDriver::getProfileCfg()
{
    ProfileCfg profileCfg;
    profileCfg.acceleration = getAcc();
    profileCfg.deceleration = getDec();
    profileCfg.maxSpeed = getMaxSpeed();
    profileCfg.minSpeed = getMinSpeed();
    return profileCfg;
}

// Setup the SYNC/BUSY pin to be either SYNC or BUSY, and to a desired
//  ticks per step level.
void AutoDriver::setSyncSelect( SyncSelect syncSelect , bool syncEnable)
{
  // Only some of the bits in this register are of interest to us; we need to
  //  clear those bits. It happens that they are the upper four.
  const uint8_t syncMask = 0x0F;
  uint8_t syncPinConfig = (uint8_t)getParam(STEP_MODE);
  syncPinConfig &= syncMask;
  
  // Now, let's OR in the arguments. We're going to mask the incoming
  //  data to avoid touching any bits that aren't appropriate. See datasheet
  //  for more info about which bits we're touching.
  const uint8_t syncEnableMask = 0x80;
  const uint8_t syncSelectMask = 0x70;
  syncPinConfig |= ((syncEnable & syncEnableMask) | (syncSelect & syncSelectMask));
  
  // Now we should be able to send that uint8_t right back to the dSPIN- it
  //  won't corrupt the other bits, and the changes are made.
  setParam(STEP_MODE, (unsigned long)syncPinConfig);
}

void
AutoDriver::setAlarmState(AlarmState alarmState)
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
    setParam(ALARM_EN,alarmStateByte);
}

AlarmState
AutoDriver::getAlarmState()
{
    AlarmState alarmState;
    uint8_t alarmStateByte = getParam(ALARM_EN);

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
AutoDriver::getSyncSelect()
{
    const uint8_t syncSelectMask = 0x70;
    return static_cast<SyncSelect>(getParam(STEP_MODE) & syncSelectMask);
}

bool
AutoDriver::getSyncEnable()
{
    const uint8_t syncEnableMask = 0x80;
    return (getParam(STEP_MODE) & syncEnableMask);
}

// The dSPIN chip supports microstepping for a smoother ride. This function
//  provides an easy front end for changing the microstepping mode.
void AutoDriver::setStepMode(StepMode stepMode)
{
  // Only some of these bits are useful (the lower three). We'll extract the
  //  current contents, clear those three bits, then set them accordingly.
  const uint8_t stepModeMask = 0xF8;
  uint8_t stepModeConfig = (uint8_t)getParam(STEP_MODE);
  stepModeConfig &= stepModeMask;
  
  // Now we can OR in the new bit settings. Mask the argument so we don't
  //  accidentally the other bits, if the user sends us a non-legit value.
  stepModeConfig |= (stepMode&STEP_MODE_STEP_SEL);
  
  // Now push the change to the chip.
  setParam(STEP_MODE, (unsigned long)stepModeConfig);
}

StepMode AutoDriver::getStepMode() {
  return static_cast<StepMode>(getParam(STEP_MODE) & STEP_MODE_STEP_SEL);
}

// This is the maximum speed the dSPIN will attempt to produce.
void AutoDriver::setMaxSpeed(float stepsPerSecond)
{
  // We need to convert the floating point stepsPerSecond into a value that
  //  the dSPIN can understand. Fortunately, we have a function to do that.
  unsigned long integerSpeed = maxSpdCalc(stepsPerSecond);
  
  // Now, we can set that paramter.
  setParam(MAX_SPEED, integerSpeed);
}

float AutoDriver::getMaxSpeed()
{
  return maxSpdParse(getParam(MAX_SPEED));
}

// Set the minimum speed allowable in the system. This is the speed a motion
//  starts with; it will then ramp up to the designated speed or the max
//  speed, using the acceleration profile.
void AutoDriver::setMinSpeed(float stepsPerSecond)
{
  // We need to convert the floating point stepsPerSecond into a value that
  //  the dSPIN can understand. Fortunately, we have a function to do that.
  unsigned long integerSpeed = minSpdCalc(stepsPerSecond);
  
  // MIN_SPEED also contains the LSPD_OPT flag, so we need to protect that.
  unsigned long temp = getParam(MIN_SPEED) & 0x00001000;
  
  // Now, we can set that paramter.
  setParam(MIN_SPEED, integerSpeed | temp);
}

float AutoDriver::getMinSpeed()
{
  return minSpdParse(getParam(MIN_SPEED));
}

// Above this threshold, the dSPIN will cease microstepping and go to full-step
//  mode. 
void AutoDriver::setFullSpeed(float stepsPerSecond)
{
  unsigned long integerSpeed = FSCalc(stepsPerSecond);
  setParam(FS_SPD, integerSpeed);
}

float AutoDriver::getFullSpeed()
{
  return FSParse(getParam(FS_SPD));
}

// Set the acceleration rate, in steps per second per second. This value is
//  converted to a dSPIN friendly value. Any value larger than 29802 will
//  disable acceleration, putting the chip in "infinite" acceleration mode.
void AutoDriver::setAcc(float stepsPerSecondPerSecond)
{
  unsigned long integerAcc = accCalc(stepsPerSecondPerSecond);
  setParam(ACC, integerAcc);
}

float AutoDriver::getAcc()
{
  return accParse(getParam(ACC));
}

// Same rules as setAcc().
void AutoDriver::setDec(float stepsPerSecondPerSecond)
{
  unsigned long integerDec = decCalc(stepsPerSecondPerSecond);
  setParam(DECEL, integerDec);
}

float AutoDriver::getDec()
{
  return accParse(getParam(DECEL));
}

void AutoDriver::setOCThreshold(CurrentThreshold ocThreshold)
{
  setParam(OCD_TH, 0x0F & ocThreshold);
}

CurrentThreshold AutoDriver::getOCThreshold()
{
  return static_cast<CurrentThreshold> (getParam(OCD_TH) & CONFIG_OC_THRESOLD_REG);
}

void
AutoDriver::setStallThreshold(CurrentThreshold stallCurrent)
{
    setParam(STALL_TH,0x0F & stallCurrent);
}

CurrentThreshold
AutoDriver::getStallThreshold()
{
    const long stallThresholdMask=0xFF; // TODO - fix!!
    return static_cast<CurrentThreshold> (getParam(STALL_TH) & stallThresholdMask);
}

// The next few functions are all breakouts for individual options within the
//  single register CONFIG. We'll read CONFIG, blank some bits, then OR in the
//  new value.

// This is a multiplier/divider setup for the PWM frequency when microstepping.
//  Divisors of 1-7 are available; multipliers of .625-2 are available. See
//  datasheet for more details; it's not clear what the frequency being
//  multiplied/divided here is, but it is clearly *not* the actual clock freq.
void AutoDriver::setPWMFreq(PwmFrequencyDivider divider, PwmFrequencyMultiplier multiplier)
{
  unsigned long configVal = getParam(CONFIG);
  
  // The divisor is set by config 15:13, so mask 0xE000 to clear them.
  configVal &= ~(CONFIG_F_PWM_INT);
  // The multiplier is set by config 12:10; mask is 0x1C00
  configVal &= ~(CONFIG_F_PWM_DEC);
  // Now we can OR in the masked-out versions of the values passed in.
  configVal |= ((CONFIG_F_PWM_INT&divider)|(CONFIG_F_PWM_DEC&multiplier));
  setParam(CONFIG, configVal);
}

PwmFrequencyDivider AutoDriver::getPWMFreqDivisor()
{
  return static_cast<PwmFrequencyDivider> (getParam(CONFIG) & CONFIG_F_PWM_INT);
}

PwmFrequencyMultiplier AutoDriver::getPWMFreqMultiplier()
{
  return static_cast<PwmFrequencyMultiplier> (getParam(CONFIG) & CONFIG_F_PWM_DEC);
}

// Slew rate of the output in V/us. Can be 180, 290, or 530.
void AutoDriver::setSlewRate(SlewRate slewRate)
{
  unsigned long configVal = getParam(CONFIG);
  
  // These bits live in CONFIG 9:8, so the mask is 0x0300.
  configVal &= ~(CONFIG_SLEW_RATE_MASK);
  //Now, OR in the masked incoming value.
  configVal |= (CONFIG_SLEW_RATE_MASK&slewRate);
  setParam(CONFIG, configVal);
}

SlewRate AutoDriver::getSlewRate()
{
  return static_cast<SlewRate> (getParam(CONFIG) & CONFIG_SLEW_RATE_MASK);
}

// Single bit- do we shutdown the drivers on overcurrent or not?
void AutoDriver::setOCShutdown(OverCurrentDetection overCurrentDetection)
{
  unsigned long configVal = getParam(CONFIG);
  // This bit is CONFIG 7, mask is 0x0080
  configVal &= ~(CONFIG_OC_DETECTION_MASK);
  //Now, OR in the masked incoming value.
  configVal |= (CONFIG_OC_DETECTION_MASK&overCurrentDetection);
  setParam(CONFIG, configVal);
}

OverCurrentDetection AutoDriver::getOCShutdown()
{
  return static_cast<OverCurrentDetection> (getParam(CONFIG) & CONFIG_OC_DETECTION_MASK);
}

// Enable motor voltage compensation? Not at all straightforward- check out
//  p34 of the datasheet.
void AutoDriver::setVoltageComp(VoltageCompensation vsCompMode)
{
  unsigned long configVal = getParam(CONFIG);
  // This bit is CONFIG 5, mask is 0x0020
  configVal &= ~(CONFIG_EN_VSCOMP_MASK);
  //Now, OR in the masked incoming value.
  configVal |= (CONFIG_EN_VSCOMP_MASK&vsCompMode);
  setParam(CONFIG, configVal);
}

VoltageCompensation AutoDriver::getVoltageComp()
{
  return static_cast <VoltageCompensation> (getParam(CONFIG) & CONFIG_EN_VSCOMP_MASK);
}

// The switch input can either hard-stop the driver _or_ activate an interrupt.
//  This bit allows you to select what it does.
void AutoDriver::setSwitchMode(SwitchConfiguration switchMode)
{
  unsigned long configVal = getParam(CONFIG);
  // This bit is CONFIG 4, mask is 0x0010
  configVal &= ~(CONFIG_SW_MODE_MASK);
  //Now, OR in the masked incoming value.
  configVal |= (CONFIG_SW_MODE_MASK & switchMode);
  setParam(CONFIG, configVal);
}

SwitchConfiguration AutoDriver::getSwitchMode()
{
  return static_cast <SwitchConfiguration> (getParam(CONFIG) & CONFIG_SW_MODE_MASK);
}

// There are a number of clock options for this chip- it can be configured to
//  accept a clock, drive a crystal or resonator, and pass or not pass the
//  clock signal downstream. Theoretically, you can use pretty much any
//  frequency you want to drive it; practically, this library assumes it's
//  being driven at 16MHz. Also, the device will use these bits to set the
//  math used to figure out steps per second and stuff like that.
void AutoDriver::setOscMode(OscillatorSelect oscillatorMode)
{
  unsigned long configVal = getParam(CONFIG);
  // These bits are CONFIG 3:0, mask is 0x000F
  configVal &= ~(CONFIG_OSC_SEL_MASK);
  //Now, OR in the masked incoming value.
  configVal |= (CONFIG_OSC_SEL_MASK&oscillatorMode);
  setParam(CONFIG, configVal);
}

OscillatorSelect AutoDriver::getOscMode()
{
  return static_cast <OscillatorSelect> (getParam(CONFIG) & CONFIG_OSC_SEL_MASK);
}

// The KVAL registers are...weird. I don't entirely understand how they differ
//  from the microstepping, but if you have trouble getting the motor to run,
//  tweaking KVAL has proven effective in the past. There's a separate register
//  for each case: running, static, accelerating, and decelerating.

void AutoDriver::setAccKVAL(uint8_t kvalInput)
{
  setParam(KVAL_ACC, kvalInput);
}

uint8_t AutoDriver::getAccKVAL()
{
  return (uint8_t) getParam(KVAL_ACC);
}

void AutoDriver::setDecKVAL(uint8_t kvalInput)
{
  setParam(KVAL_DEC, kvalInput);
}

uint8_t AutoDriver::getDecKVAL()
{
  return (uint8_t) getParam(KVAL_DEC);
}

void AutoDriver::setRunKVAL(uint8_t kvalInput)
{
  setParam(KVAL_RUN, kvalInput);
}

uint8_t AutoDriver::getRunKVAL()
{
  return (uint8_t) getParam(KVAL_RUN);
}

void AutoDriver::setHoldKVAL(uint8_t kvalInput)
{
  setParam(KVAL_HOLD, kvalInput);
}

uint8_t AutoDriver::getHoldKVAL()
{
  return (uint8_t) getParam(KVAL_HOLD);
}

void
AutoDriver::setBackEmfConfig(const BackEmfConfig &backEmfConfig)
{
    // Set the K Values
    setHoldKVAL(backEmfConfig.holdingKVal);
    setRunKVAL(backEmfConfig.constantSpeedKVal);
    setAccKVAL(backEmfConfig.accelStartingKVal);
    setDecKVAL(backEmfConfig.decelStartingKVal);

    // Set the intersect speed and slope of the curve
    setParam(INT_SPD,backEmfConfig.intersectSpeed);
    setParam(ST_SLP,backEmfConfig.startSlope);
    setParam(FN_SLP_ACC,backEmfConfig.accelFinalSlope);
    setParam(FN_SLP_DEC,backEmfConfig.decelFinalSlope);
}

BackEmfConfig
AutoDriver::getBackEmfConfig()
{
    BackEmfConfig backEmfConfig;

    // Get the K Values
    backEmfConfig.holdingKVal = getHoldKVAL();
    backEmfConfig.constantSpeedKVal = getRunKVAL();
    backEmfConfig.accelStartingKVal = getAccKVAL();
    backEmfConfig.decelStartingKVal = getDecKVAL();

    // Get the intersect speed and slope of the curve
    backEmfConfig.intersectSpeed  = getParam(INT_SPD);
    backEmfConfig.startSlope      = getParam(ST_SLP);
    backEmfConfig.accelFinalSlope = getParam(FN_SLP_ACC);
    backEmfConfig.decelFinalSlope = getParam(FN_SLP_DEC);

    return backEmfConfig;
}

// Enable or disable the low-speed optimization option. With LSPD_OPT enabled,
//  motion starts from 0 instead of MIN_SPEED and low-speed optimization keeps
//  the driving sine wave prettier than normal until MIN_SPEED is reached.
void AutoDriver::setLoSpdOpt(bool enable)
{
  unsigned long temp = getParam(MIN_SPEED);
  if (enable) temp |= 0x00001000; // Set the LSPD_OPT bit
  else        temp &= 0xffffefff; // Clear the LSPD_OPT bit
  setParam(MIN_SPEED, temp);
}

bool AutoDriver::getLoSpdOpt()
{
  return (bool) ((getParam(MIN_SPEED) & 0x00001000) != 0);
}

