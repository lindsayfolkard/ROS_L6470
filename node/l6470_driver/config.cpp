#include "config.h"
#include <fstream>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/bimap.hpp>
#include "types.h"

AbstractConfig::~AbstractConfig(){}

WriteableConfig::~WriteableConfig(){}

std::string toString(MotorDriverType motorDriverType)
{
    switch (motorDriverType)
    {
    case PowerStep01:
        return "PowerStep01";
    case L6470:
        return "L6470";
    case L6472:
        return "L6472";
    case Simulator:
        return "Simulator";
    default:
        assert(!"Invalid MotorDriverType in toString");
    }
}

MotorDriverType motorDriverTypeFromString(const std::string &str)
{
    if (str == toString(PowerStep01))
        return PowerStep01;
    else if (str == toString(L6470))
        return L6470;
    else if (str == toString(L6472))
        return L6472;
    else if (str == toString(Simulator))
        return Simulator;
    else
        throw; // TODO - add some more information
}
/////////////////////////////////
//// Overall Config /////////////
/////////////////////////////////
///
///

namespace pt = boost::property_tree;

OverallCfg::OverallCfg(const std::string &filePath)
{
    // Let's do this in json format (easier to parse)
    pt::ptree root;

    try
    {
        pt::read_json(filePath,root);
        controllerType_  = motorDriverTypeFromString(root.get<std::string>("controllerType"));
        commsDebugLevel_ = static_cast<CommsDebugLevel>(root.get<int>("commsDebugLevel"));
        spiBus_          = root.get<int>("spiBus");

        // Get the stepper motor configs
        for (pt::ptree::value_type &child : root.get_child("motors"))
        {
            CfgFile cfg(child.second.get<std::string> ("motorCfgFile"),
                        child.second.get<std::string> ("customCfgFile"),
                        child.second.get<std::string> ("model"));
            cfgFiles_.push_back(cfg);
        }

    }
    catch (std::exception &e)
    {
        std::cout << "Exception thrown while trying to read overallCfg with reason " << e.what();
        throw; // rethrow the exception
    }
}

OverallCfg::OverallCfg( const std::vector<CfgFile> &cfgFiles,
                        MotorDriverType                 controllerType,
                        CommsDebugLevel                 commsDebugLevel,
                        int                             spiBus):
    cfgFiles_(cfgFiles),
    controllerType_(controllerType),
    commsDebugLevel_(commsDebugLevel),
    spiBus_(spiBus)
{}

void
OverallCfg::writeToFile(const std::string &baseFile)
{
    pt::ptree root;

    root.put("controllerType",toString(controllerType_));
    root.put("commsDebugLevel",(int)commsDebugLevel_);
    root.put("spiBus",spiBus_);

    pt::ptree cfgNode;

    for (const auto &cfgFile : cfgFiles_)
    {
        pt::ptree child;
        child.put("model",cfgFile.motorModel_);
        child.put("motorCfgFile",cfgFile.stepperMotorFile_);
        child.put("customCfgFile",cfgFile.commonConfigFile_);
        cfgNode.push_back((std::make_pair("",child)));
    }

    root.add_child("motors",cfgNode);

    // Write to file
    std::ofstream outFile;
    outFile.open(baseFile);
        //throw; // TODO - fix to real exception
    pt::write_json(outFile,root);
}

std::string
toString(const OverallCfg &cfg)
{
    std::stringstream ss;
    ss << "ControllerType  : " << cfg.controllerType_  << std::endl;
    ss << "CommsDebugLevel : " << cfg.commsDebugLevel_ << std::endl;

    int count=1;
    for (const auto &file : cfg.cfgFiles_)
    {
        ss << "Motor " << count << " : " << file.stepperMotorFile_ << " , " << file.commonConfigFile_ << std::endl;
        ++count;
    }

    return ss.str();
}

//////////////////////////////////
/// Common Configuration Commands
//////////////////////////////////

void 
CommonConfig::setDefaults()
{
    fullStepThresholdSpeed=900;
    thermalDriftCoefficient=0;

    overCurrentThreshold=OCD_TH_6000m;
    stallThreshold=OCD_TH_6000m;

    // STEP_MODE register settings
    stepMode=STEP_SEL_1_16;
    syncSelect=SYNC_SEL_16;
    controlMode=VoltageControlMode; // Voltage or current (NB: some chips will generally support one or the other)
    syncEnable=true;

    // CONFIG register settings
    oscillatorSelect=CONFIG_INT_16MHZ;
    switchConfiguration=CONFIG_SW_HARD_STOP;
    overCurrentDetection=CONFIG_OC_SD_ENABLE;

    // Alarm Register Settings
    alarmState.overCurrentEnabled=true;
    alarmState.thermalShutdownEnabled=true;
    alarmState.thermalWarningEnabled=true;
    alarmState.underVoltageEnabled=true;
    alarmState.stallDetectionAEnabled=true;
    alarmState.stallDetectionBEnabled=true;
    alarmState.switchTurnOnEnabled=true;
    alarmState.badCommandEnabled=true;
}

void
CommonConfig::set(CommsDriver &commsDriver, int motor)
{
    //checkMotorIsValid(motor);

    // Current Thresholds
    setOCThreshold(overCurrentThreshold, commsDriver, motor);
    usleep(1000);
    setStallThreshold(stallThreshold, commsDriver, motor);
    usleep(1000);
    setOCShutdown(overCurrentDetection, commsDriver, motor);
    usleep(1000);

    // Step Mode
    setStepMode(stepMode, commsDriver, motor);
    usleep(1000);
    setSyncSelect(syncSelect, syncEnable, commsDriver, motor);
    usleep(1000);

    // Set Oscillator related configs
    setOscMode(oscillatorSelect, commsDriver, motor);
    usleep(1000);
    setSwitchMode(switchConfiguration, commsDriver, motor);
    usleep(1000);

    // Set Alarm State
    setAlarmState(alarmState, commsDriver, motor);
    usleep(1000);

    commsDriver.setParam(FS_SPD,toBitLength(FS_SPD),fullStepThresholdSpeed,motor);
    usleep(1000);

//    // Set Full Step Threshold speed
    commsDriver.SPIXfer(0,0);
    commsDriver.SPIXfer(0,0);
    commsDriver.SPIXfer(0,0);
    commsDriver.SPIXfer(0,0);
    commsDriver.SPIXfer(0,0);
    commsDriver.SPIXfer(0,0);
//    usleep(5000);
//    std::cout << "Debug - Lets try to set the FS_SPD parameter" << fullStepThresholdSpeed << std::endl;
//    commsDriver.setParam(FS_SPD,toBitLength(FS_SPD),fullStepThresholdSpeed,motor);
//    commsDriver.SPIXfer(0,0);
//    commsDriver.SPIXfer(0,0);
//    commsDriver.SPIXfer(0,0);
//    commsDriver.SPIXfer(0,0);
//    commsDriver.SPIXfer(0,0);
//    commsDriver.SPIXfer(0,0);
//    usleep(5000);
//    std::cout << "Debug - let's do some more shit" << std::endl;
//    std::cout << "I think I have set the parameter" << std::endl;
//    std::cout << "Get the parameter" << std::endl;
//    std::cout << commsDriver.getParam(FS_SPD,toBitLength(FS_SPD),motor) << std::endl;
//    commsDriver.SPIXfer(0,0);
//    commsDriver.SPIXfer(0,0);
//    commsDriver.SPIXfer(0,0);
//    commsDriver.SPIXfer(0,0);
//    commsDriver.SPIXfer(0,0);
//    commsDriver.SPIXfer(0,0);
//    usleep(5000);
//    std::cout << commsDriver.getParam(FS_SPD,toBitLength(FS_SPD),motor) << std::endl;

}

enum Colour {
    Red,
    Blue,
    Green,
    Yellow,
    Orange
};

std::string addColour(const std::string &str, Colour colour)
{
    if (colour == Red)
        return "\033[31m" + str + "\033[0m";
    else if (colour == Green)
        return "\033[32m" + str + "\033[0m";
    else if (colour == Blue)
        return "\033[34m" + str + "\033[0m";
    else if (colour == Yellow)
        return "\033[33m" + str + "\033[0m";
    else if (colour == Orange)
        return "\033[35m"  + str + "\033[0m";
    else
        return str;
}

template <typename T> void testAllCombinations (CommsDriver &commsDriver, int motor , boost::bimap<T,std::string> &biMap ,
                                                const std::string &paramDescr,
                                                std::function<void(T value, CommsDriver &commsDriver, int motor)> setFunction ,
                                                std::function<T (CommsDriver &commsDriver,int motor)> getFunction)
{
    bool passed=true;
    std::cout << "IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII" << std::endl;
    std::cout << "Test commms for setting " << paramDescr << std::endl;
    std::cout << "IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII" << std::endl;

    for (auto it = biMap.begin(); it != biMap.end() ; ++it)
    {
        std::cout << "Set to " << it->right << " , ";
        setFunction(it->left,commsDriver,motor);
        const T value = getFunction(commsDriver,motor);
        //commsDriver.setParam(paramRegister,toBitLength(paramRegister),it->left,motor);
        //const T value = static_cast<T>(commsDriver.getParam(paramRegister,toBitLength(paramRegister),motor));
        std::cout << "read " << value;
        if (value != it->left)
        {
            passed=false;
            std::cout << " : " << addColour("Failed!",Red) << std::endl;
        }
        else
        {
            std::cout << " : " << addColour("Passed!" ,Green)<< std::endl;
        }
    }

    std::cout << "Test for setting " << paramDescr << " : ";
    if (passed)
        std::cout << addColour("Passed",Green) << std::endl;
    else
        std::cout << addColour("Failed!",Red) << std::endl;
    std::cout << "IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII" << std::endl << std::endl;

}

void testAllCombinations (CommsDriver &driver, int motor , ParamRegister paramRegister)
{
    bool passed=true;
    std::cout << "IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII" << std::endl;
    std::cout << "Test commms for setting " << paramRegister << std::endl;
    std::cout << "IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII" << std::endl;

    const int maxVal = pow(2,toBitLength(paramRegister));
    for (int i = 0 ; i < maxVal ; ++i)
    {
        std::cout << "Set to " << i << " , ";
        driver.setParam(paramRegister,toBitLength(paramRegister),i,motor);
        usleep(1000);
        int value = driver.getParam(paramRegister,toBitLength(paramRegister),motor);
        std::cout << "read " << (int) value;
        if (value != i)
        {
            passed=false;
            std::cout << " : " << addColour("Failed!",Red) << std::endl;
        }
        else
        {
            std::cout << " : " << addColour("Passed!" ,Green)<< std::endl;
        }
        usleep(1000);
    }

    std::cout << "Test for setting " << paramRegister << " : ";
    if (passed)
        std::cout << addColour("Passed",Green) << std::endl;
    else
        std::cout << addColour("Failed!",Red) << std::endl;
    std::cout << "IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII" << std::endl << std::endl;
}

void CommonConfig::unitTest(CommsDriver &commsDriver, int motor)
{

    // OverCurrentThreshold
    // Let's just manually do this shit...
    boost::bimap <CurrentThreshold,std::string> currentThresholdMap = getCurrentThresholdBiMap();
    testAllCombinations<CurrentThreshold>(commsDriver,motor,currentThresholdMap,"OCThreshold",setOCThreshold,getOCThreshold);

    // StallThreshold
    testAllCombinations<CurrentThreshold>(commsDriver,motor,currentThresholdMap,"StallThreshold",setStallThreshold,getStallThreshold);

    // StepMode
    boost::bimap<StepMode,std::string> stepModeBimap = getStepModeBiMap();
    testAllCombinations<StepMode>(commsDriver,motor,stepModeBimap,"StepMode",setStepMode,getStepMode);

    // SyncSelect
    boost::bimap<SyncSelect,std::string> syncSelectBimap = getSyncSelectBiMap();
    std::function <void (SyncSelect s, CommsDriver &commsDriver, int motor)> setSync= [] (SyncSelect s, CommsDriver &commsDriver, int motor ) { return setSyncSelect(s,true,commsDriver,motor);};
    testAllCombinations<SyncSelect>(commsDriver,motor,syncSelectBimap,"SyncSelect",setSync,getSyncSelect);

    // OscillatorSelect
    boost::bimap<OscillatorSelect,std::string> oscillatorSelectMap = getOscillatorSelectBiMap();
    testAllCombinations<OscillatorSelect>(commsDriver,motor,oscillatorSelectMap,"OscillatorSelect",setOscMode,getOscMode);

    // Switch Configuration

    // AlarmState ?

    // FullStepThreshold Speed
    boost::bimap<int,std::string> fsMap = makeBiMap<int,std::string>({{100,"100"},{200,"200"},{300,"300"},{400,"400"},{500,"500"},{600,"600"},{700,"700"}});
    testAllCombinations<int>(commsDriver,motor,fsMap,"FullStepSpeed",setFullSpeed,getFullSpeed);

}

CommonConfig::CommonConfig(CommsDriver &commsDriver , int motor)
{
    overCurrentThreshold = getOCThreshold(commsDriver , motor);
    overCurrentDetection = getOCShutdown(commsDriver , motor);
    stallThreshold       = getStallThreshold(commsDriver , motor);

    stepMode             = getStepMode(commsDriver , motor);
    syncSelect           = getSyncSelect(commsDriver , motor);
    syncEnable           = getSyncEnable(commsDriver , motor);

    oscillatorSelect         = getOscMode(commsDriver , motor);
    switchConfiguration      = getSwitchMode(commsDriver , motor);
    alarmState               = getAlarmState(commsDriver , motor);
    //thermalDriftCoefficient = getTh
    fullStepThresholdSpeed  = commsDriver.getParam(FS_SPD,toBitLength(FS_SPD),motor);
}

// Setup the SYNC/BUSY pin to be either SYNC or BUSY, and to a desired
//  ticks per step level.
void
CommonConfig::setSyncSelect( SyncSelect syncSelect, bool syncEnable, CommsDriver &commsDriver, int motor)
{
    // Only some of the bits in this register are of interest to us; we need to
    //  clear those bits. It happens that they are the upper four.
    const uint8_t syncMask = 0x0F;
    uint8_t syncPinConfig = (uint8_t)commsDriver.getParam(STEP_MODE,toBitLength(STEP_MODE),motor);
    syncPinConfig &= syncMask;

    // Now, let's OR in the arguments. We're going to mask the incoming
    //  data to avoid touching any bits that aren't appropriate. See datasheet
    //  for more info about which bits we're touching.
    const uint8_t syncEnableMask = 0x80;
    const uint8_t syncSelectMask = 0x70;
    syncPinConfig |= ((syncEnable & syncEnableMask) | (syncSelect & syncSelectMask));

    // Now we should be able to send that uint8_t right back to the dSPIN- it
    //  won't corrupt the other bits, and the changes are made.
    commsDriver.setParam(STEP_MODE, toBitLength(STEP_MODE),(uint32_t)syncPinConfig , motor);
}

void
CommonConfig::setAlarmState(AlarmState alarmState, CommsDriver &commsDriver, int motor)
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
    commsDriver.setParam(ALARM_EN, toBitLength(alarmStateByte),alarmStateByte, motor);
}

AlarmState
CommonConfig::getAlarmState(CommsDriver &commsDriver, int motor)
{
    AlarmState alarmState;
    uint8_t alarmStateByte = commsDriver.getParam(ALARM_EN, toBitLength(ALARM_EN), motor);

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
CommonConfig::getSyncSelect(CommsDriver &commsDriver, int motor)
{
    const uint8_t syncSelectMask = 0x70;
    return static_cast<SyncSelect>(commsDriver.getParam(STEP_MODE, toBitLength(STEP_MODE), motor) & syncSelectMask);
}

bool
CommonConfig::getSyncEnable(CommsDriver &commsDriver, int motor)
{
    const uint8_t syncEnableMask = 0x80;
    return (commsDriver.getParam(STEP_MODE, toBitLength(STEP_MODE), motor) & syncEnableMask);
}

// The dSPIN chip supports microstepping for a smoother ride. This function
//  provides an easy front end for changing the microstepping mode.
void
CommonConfig::setStepMode(StepMode stepMode, CommsDriver &commsDriver, int motor)
{
    // Only some of these bits are useful (the lower three). We'll extract the
    //  current contents, clear those three bits, then set them accordingly.
    const uint8_t stepModeMask = 0xF8;
    uint8_t stepModeConfig = (uint8_t)commsDriver.getParam(STEP_MODE,toBitLength(STEP_MODE),motor);
    stepModeConfig &= stepModeMask;

    // Now we can OR in the new bit settings. Mask the argument so we don't
    //  accidentally the other bits, if the user sends us a non-legit value.
    stepModeConfig |= (stepMode&STEP_MODE_STEP_SEL);

    // Now push the change to the chip.
    commsDriver.setParam(STEP_MODE, toBitLength(STEP_MODE), (uint32_t)stepModeConfig , motor);
}

StepMode
CommonConfig::getStepMode(CommsDriver &commsDriver, int motor) {
    return static_cast<StepMode>(commsDriver.getParam(STEP_MODE, toBitLength(STEP_MODE), motor) & STEP_MODE_STEP_SEL);
}

// Above this threshold, the dSPIN will cease microstepping and go to full-step
//  mode.
void
CommonConfig::setFullSpeed(float stepsPerSecond, CommsDriver &commsDriver, int motor)
{
    uint32_t integerSpeed = FSCalc(stepsPerSecond);
    commsDriver.setParam(FS_SPD, toBitLength(FS_SPD), integerSpeed, motor);
}


int
CommonConfig::getFullSpeed(CommsDriver &commsDriver, int motor)
{
    commsDriver.getParam(FS_SPD, toBitLength(FS_SPD), motor);
    return FSParse(commsDriver.getParam(FS_SPD, toBitLength(FS_SPD), motor));
}

//float
//CommonConfig::getFullSpeed(CommsDriver &commsDriver, int motor)
//{
//  return FSParse(commsDriver.getParam(FS_SPD, toBitLength(FS_SPD), motor));
//}

void
CommonConfig::setOCThreshold(CurrentThreshold ocThreshold, CommsDriver &commsDriver, int motor)
{
    //std::cout << "Debug - try to set ocThreshold to " << ocThreshold << std::endl;
    //commsDriver.setParam(OCD_TH, toBitLength(OCD_TH), 0x0F & ocThreshold, motor);
    commsDriver.setParam(OCD_TH, toBitLength(OCD_TH), ocThreshold, motor);
    //std::cout << "Debug - read new ocThreshold as " << commsDriver.getParam(OCD_TH,toBitLength(OCD_TH),motor) << std::endl;
}

CurrentThreshold
CommonConfig::getOCThreshold(CommsDriver &commsDriver, int motor)
{
    return static_cast<CurrentThreshold> (commsDriver.getParam(OCD_TH, toBitLength(OCD_TH), motor));
}

void
CommonConfig::setStallThreshold(CurrentThreshold stallCurrent, CommsDriver &commsDriver, int motor)
{
    commsDriver.setParam(STALL_TH, toBitLength(STALL_TH), stallCurrent, motor);
}

CurrentThreshold
CommonConfig::getStallThreshold(CommsDriver &commsDriver, int motor)
{
    const uint32_t stallThresholdMask=0xFF; // TODO - fix!!
    return static_cast<CurrentThreshold> (commsDriver.getParam(STALL_TH, toBitLength(STALL_TH), motor));
}

// Single bit- do we shutdown the drivers on overcurrent or not?
void
CommonConfig::setOCShutdown(OverCurrentDetection overCurrentDetection, CommsDriver &commsDriver, int motor)
{
    uint32_t configVal = commsDriver.getParam(CONFIG, toBitLength(CONFIG), motor);
    // This bit is CONFIG 7, mask is 0x0080
    configVal &= ~(CONFIG_OC_DETECTION_MASK);
    //Now, OR in the masked incoming value.
    configVal |= (CONFIG_OC_DETECTION_MASK & overCurrentDetection);
    commsDriver.setParam(CONFIG, toBitLength(CONFIG), configVal, motor);
}

OverCurrentDetection
CommonConfig::getOCShutdown(CommsDriver &commsDriver, int motor)
{
    return static_cast<OverCurrentDetection> (commsDriver.getParam(CONFIG, toBitLength(CONFIG), motor) & CONFIG_OC_DETECTION_MASK);
}

// The switch input can either hard-stop the driver _or_ activate an interrupt.
//  This bit allows you to select what it does.
void
CommonConfig::setSwitchMode(SwitchConfiguration switchMode, CommsDriver &commsDriver, int motor)
{
    uint32_t configVal = commsDriver.getParam(CONFIG, toBitLength(CONFIG), motor);
    // This bit is CONFIG 4, mask is 0x0010
    configVal &= ~(CONFIG_SW_MODE_MASK);
    //Now, OR in the masked incoming value.
    configVal |= (CONFIG_SW_MODE_MASK & switchMode);
    commsDriver.setParam(CONFIG, toBitLength(CONFIG), configVal, motor);
}

SwitchConfiguration
CommonConfig::getSwitchMode(CommsDriver &commsDriver, int motor)
{
    return static_cast <SwitchConfiguration> (commsDriver.getParam(CONFIG, toBitLength(CONFIG), motor) & CONFIG_SW_MODE_MASK);
}

// There are a number of clock options for this chip- it can be configured to
//  accept a clock, drive a crystal or resonator, and pass or not pass the
//  clock signal downstream. Theoretically, you can use pretty much any
//  frequency you want to drive it; practically, this library assumes it's
//  being driven at 16MHz. Also, the device will use these bits to set the
//  math used to figure out steps per second and stuff like that.
void
CommonConfig::setOscMode(OscillatorSelect oscillatorMode, CommsDriver &commsDriver, int motor)
{
    uint32_t configVal = commsDriver.getParam(CONFIG, toBitLength(CONFIG), motor);
    // These bits are CONFIG 3:0, mask is 0x000F
    configVal &= ~(CONFIG_OSC_SEL_MASK);
    //Now, OR in the masked incoming value.
    configVal |= (CONFIG_OSC_SEL_MASK&oscillatorMode);
    commsDriver.setParam(CONFIG, toBitLength(CONFIG), configVal, motor);
}

OscillatorSelect
CommonConfig::getOscMode(CommsDriver &commsDriver, int motor)
{
    return static_cast <OscillatorSelect> (commsDriver.getParam(CONFIG, toBitLength(CONFIG), motor) & CONFIG_OSC_SEL_MASK);
}

//////////////////////////////////////////////////
//// END Common Config Functions
//////////////////////////////////////////////////



//////////////////////////////////////////////////
//// Voltage Mode Config Functions
//////////////////////////////////////////////////


// The next few functions are all breakouts for individual options within the
//  single register CONFIG. We'll read CONFIG, blank some bits, then OR in the
//  new value.

// This is a multiplier/divider setup for the PWM frequency when microstepping.
//  Divisors of 1-7 are available; multipliers of .625-2 are available. See
//  datasheet for more details; it's not clear what the frequency being
//  multiplied/divided here is, but it is clearly *not* the actual clock freq.
void
VoltageModeCfg::setPWMFreq(PwmFrequencyDivider divider, PwmFrequencyMultiplier multiplier, CommsDriver &commsDriver, int motor)
{
    uint32_t configVal = commsDriver.getParam(CONFIG, toBitLength(CONFIG), motor);

    // The divisor is set by config 15:13, so mask 0xE000 to clear them.
    configVal &= ~(CONFIG_F_PWM_INT);
    // The multiplier is set by config 12:10; mask is 0x1C00
    configVal &= ~(CONFIG_F_PWM_DEC);
    // Now we can OR in the masked-out versions of the values passed in.
    configVal |= ((CONFIG_F_PWM_INT&divider)|(CONFIG_F_PWM_DEC&multiplier));
    commsDriver.setParam(CONFIG, toBitLength(CONFIG), configVal , motor);
}

void
VoltageModeCfg::setPWMFreqDivider(PwmFrequencyDivider divider, CommsDriver &commsDriver, int motor)
{
    PwmFrequencyMultiplier pwmFrequencyMultiplier = getPWMFreqMultiplier(commsDriver,motor);
    setPWMFreq(divider,pwmFrequencyMultiplier,commsDriver,motor);
}
void
VoltageModeCfg::setPWMFreqMultiplier(PwmFrequencyMultiplier multiplier, CommsDriver &commsDriver, int motor)
{
    PwmFrequencyDivider divider = getPWMFreqDivisor(commsDriver,motor);
    setPWMFreq(divider,multiplier,commsDriver,motor);
}

PwmFrequencyDivider
VoltageModeCfg::getPWMFreqDivisor(CommsDriver &commsDriver, int motor)
{
    return static_cast<PwmFrequencyDivider> (commsDriver.getParam(CONFIG, toBitLength(CONFIG), motor) & CONFIG_F_PWM_INT);
}

PwmFrequencyMultiplier
VoltageModeCfg::getPWMFreqMultiplier(CommsDriver &commsDriver, int motor)
{
    return static_cast<PwmFrequencyMultiplier> (commsDriver.getParam(CONFIG, toBitLength(CONFIG), motor) & CONFIG_F_PWM_DEC);
}

// Slew rate of the output in V/us. Can be 180, 290, or 530.
void
VoltageModeCfg::setSlewRate(SlewRate slewRate, CommsDriver &commsDriver, int motor)
{
    uint32_t configVal = commsDriver.getParam(CONFIG, toBitLength(CONFIG), motor);

    // These bits live in CONFIG 9:8, so the mask is 0x0300.
    configVal &= ~(CONFIG_SLEW_RATE_MASK);
    //Now, OR in the masked incoming value.
    configVal |= (CONFIG_SLEW_RATE_MASK&slewRate);
    commsDriver.setParam(CONFIG, toBitLength(CONFIG), configVal, motor);
}

SlewRate
VoltageModeCfg::getSlewRate(CommsDriver &commsDriver, int motor)
{
    return static_cast<SlewRate> (commsDriver.getParam(CONFIG, toBitLength(CONFIG), motor) & CONFIG_SLEW_RATE_MASK);
}

// Enable motor voltage compensation? Not at all straightforward- check out
//  p34 of the datasheet.
void
VoltageModeCfg::setVoltageComp(VoltageCompensation vsCompMode, CommsDriver &commsDriver, int motor)
{
    uint32_t configVal = commsDriver.getParam(CONFIG, toBitLength(CONFIG), motor);
    // This bit is CONFIG 5, mask is 0x0020
    configVal &= ~(CONFIG_EN_VSCOMP_MASK);
    //Now, OR in the masked incoming value.
    configVal |= (CONFIG_EN_VSCOMP_MASK&vsCompMode);
    commsDriver.setParam(CONFIG, toBitLength(CONFIG), configVal, motor);
}

VoltageCompensation
VoltageModeCfg::getVoltageComp(CommsDriver &commsDriver, int motor)
{
    return static_cast <VoltageCompensation> (commsDriver.getParam(CONFIG, toBitLength(CONFIG), motor) & CONFIG_EN_VSCOMP_MASK);
}

// The KVAL registers are...weird. I don't entirely understand how they differ
//  from the microstepping, but if you have trouble getting the motor to run,
//  tweaking KVAL has proven effective in the past. There's a separate register
//  for each case: running, static, accelerating, and decelerating.

void
VoltageModeCfg::setAccKVAL(uint8_t kvalInput, CommsDriver &commsDriver, int motor)
{
    commsDriver.setParam(KVAL_ACC, toBitLength(KVAL_ACC), kvalInput, motor);
}

uint8_t
VoltageModeCfg::getAccKVAL(CommsDriver &commsDriver, int motor)
{
    return (uint8_t) commsDriver.getParam(KVAL_ACC, toBitLength(KVAL_ACC), motor);
}

void
VoltageModeCfg::setDecKVAL(uint8_t kvalInput, CommsDriver &commsDriver, int motor)
{
    commsDriver.setParam(KVAL_DEC, toBitLength(KVAL_DEC), kvalInput, motor);
}

uint8_t
VoltageModeCfg::getDecKVAL(CommsDriver &commsDriver, int motor)
{
    return (uint8_t) commsDriver.getParam(KVAL_DEC, toBitLength(KVAL_DEC), motor);
}

void
VoltageModeCfg::setRunKVAL(uint8_t kvalInput, CommsDriver &commsDriver, int motor)
{
    commsDriver.setParam(KVAL_RUN, toBitLength(KVAL_RUN),kvalInput, motor);
}

uint8_t
VoltageModeCfg::getRunKVAL(CommsDriver &commsDriver, int motor)
{
    return (uint8_t) commsDriver.getParam(KVAL_RUN, toBitLength(KVAL_RUN), motor);
}

void
VoltageModeCfg::setHoldKVAL(uint8_t kvalInput, CommsDriver &commsDriver, int motor)
{
    commsDriver.setParam(KVAL_HOLD, toBitLength(KVAL_HOLD), kvalInput, motor);
}

uint8_t
VoltageModeCfg::getHoldKVAL(CommsDriver &commsDriver, int motor)
{
    return (uint8_t) commsDriver.getParam(KVAL_HOLD, toBitLength(KVAL_HOLD), motor);
}

void
VoltageModeCfg::set(CommsDriver &commsDriver, int motor)
{
    // Set the K Values
    setHoldKVAL(holdingKVal, commsDriver, motor);
    setRunKVAL(constantSpeedKVal, commsDriver, motor);
    setAccKVAL(accelStartingKVal, commsDriver, motor);
    setDecKVAL(decelStartingKVal, commsDriver, motor);

    // Set the intersect speed and slope of the curve
    commsDriver.setParam(INT_SPD,toBitLength(INT_SPD),intersectSpeed, motor);
    commsDriver.setParam(ST_SLP,toBitLength(INT_SPD),startSlope, motor);
    commsDriver.setParam(FN_SLP_ACC,toBitLength(FN_SLP_ACC),accelFinalSlope, motor);
    commsDriver.setParam(FN_SLP_DEC,toBitLength(FN_SLP_DEC),decelFinalSlope, motor);

    // PWM Configs
    setPWMFreq(pwmFrequencyDivider, pwmFrequencyMultiplier, commsDriver, motor);
    setSlewRate(slewRate, commsDriver, motor);
    setVoltageComp(voltageCompensation, commsDriver, motor);
}

boost::bimap<uint8_t,std::string> makeNumBiMap(uint8_t startVal , uint8_t endVal)
{
    boost::bimap<uint8_t,std::string> retVal;
    for (int i = startVal ; i <= endVal ; ++i)
    {
        retVal.insert({i, std::to_string(i)});
    }
    return retVal;
}

void
VoltageModeCfg::unitTest(CommsDriver &commsDriver, int motor)
{
    // PWM Frequency Multiplier
    boost::bimap <PwmFrequencyMultiplier,std::string> pwmMultBiMap = getPwmFrequencyMultiplierBiMap();
    testAllCombinations<PwmFrequencyMultiplier> (commsDriver,motor,pwmMultBiMap,"PWMFrequencyMultipler",setPWMFreqMultiplier,getPWMFreqMultiplier);

    // PWM Frequency Divider
    boost::bimap <PwmFrequencyDivider,std::string> pwmDivBiMap = getPwmFrequencyDividerBiMap();
    testAllCombinations<PwmFrequencyDivider> (commsDriver,motor,pwmDivBiMap,"PWMFrequencyDivider",setPWMFreqDivider,getPWMFreqDivisor);

    // Slew Rate
    boost::bimap<SlewRate,std::string> slewRateBiMap = getSlewRateBiMap();
    testAllCombinations<SlewRate> (commsDriver,motor,slewRateBiMap,"SlewRate",setSlewRate,getSlewRate);

    // Voltage Compensation
    boost::bimap <VoltageCompensation,std::string> voltageCompBimap = getVoltageCompensationBiMap();
    testAllCombinations<VoltageCompensation> (commsDriver,motor,voltageCompBimap,"VoltageComp",setVoltageComp,getVoltageComp);

    // KVal's
    boost::bimap <uint8_t,std::string> defaultBiMap = makeNumBiMap(0x00,0xFF);
    testAllCombinations<uint8_t>(commsDriver,motor,defaultBiMap,"HoldingKVal",setHoldKVAL,getHoldKVAL);
    testAllCombinations<uint8_t>(commsDriver,motor,defaultBiMap,"ConstantSpeedKVal",setRunKVAL,getRunKVAL);
    testAllCombinations<uint8_t>(commsDriver,motor,defaultBiMap,"AccelStartingKVal",setAccKVAL,getAccKVAL);
    testAllCombinations<uint8_t>(commsDriver,motor,defaultBiMap,"DecelStartingKVal",setDecKVAL,getDecKVAL);

    // Other Params
    testAllCombinations(commsDriver,motor,INT_SPD);
    testAllCombinations(commsDriver,motor,ST_SLP);
    testAllCombinations(commsDriver,motor,FN_SLP_ACC);
    testAllCombinations(commsDriver,motor,FN_SLP_DEC);
}

//VoltageModeCfg
//VoltageModeCfg::getBackEmfConfig(int motor)
//{
//    VoltageModeCfg backEmfConfig;

//    // Get the K Values
//    backEmfConfig.holdingKVal = getHoldKVAL(motor);
//    backEmfConfig.constantSpeedKVal = getRunKVAL(motor);
//    backEmfConfig.accelStartingKVal = getAccKVAL(motor);
//    backEmfConfig.decelStartingKVal = getDecKVAL(motor);

//    // Get the intersect speed and slope of the curve
//    backEmfConfig.intersectSpeed  = getParam(INT_SPD, motor);
//    backEmfConfig.startSlope      = getParam(ST_SLP, motor);
//    backEmfConfig.accelFinalSlope = getParam(FN_SLP_ACC, motor);
//    backEmfConfig.decelFinalSlope = getParam(FN_SLP_DEC, motor);

//    return backEmfConfig;
//}

// Enable or disable the low-speed optimization option. With LSPD_OPT enabled,
//  motion starts from 0 instead of MIN_SPEED and low-speed optimization keeps
//  the driving sine wave prettier than normal until MIN_SPEED is reached.
void
CommonConfig::setLoSpdOpt(bool enable, CommsDriver &commsDriver, int motor)
{
    uint32_t temp = commsDriver.getParam(MIN_SPEED, toBitLength(MIN_SPEED), motor);
    if (enable) temp |= 0x00001000; // Set the LSPD_OPT bit
    else        temp &= 0xffffefff; // Clear the LSPD_OPT bit
    commsDriver.setParam(MIN_SPEED, temp, toBitLength(MIN_SPEED), motor);
}

bool
CommonConfig::getLoSpdOpt(CommsDriver &commsDriver, int motor)
{
    return (bool) ((commsDriver.getParam(MIN_SPEED, toBitLength(MIN_SPEED), motor) & 0x00001000) != 0);
}


///////////////////////////////////////////////////////////////////
/////////// END Of Voltage Mode Config ////////////////////////////
///////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////
/////////// Start of Current Mode Config //////////////////////////
///////////////////////////////////////////////////////////////////

//
// TODO !!!
//

void
CurrentModeCfg::set(CommsDriver &commsDriver, int motor)
{
    // TODO !!!
    assert(!"set is not currently implemented for CurrentModeCfg");
}


void
CurrentModeCfg::unitTest(CommsDriver &commsDriver, int motor)
{
    assert(!"Current mode config unit test not yet implemented");
    // do nothing
}
///////////////////////////////////////////////////////////////////
//////////// END of Current Mode Config ///////////////////////////
///////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////
/////////// Parsing Related Functions /////////////////////////////
///////////////////////////////////////////////////////////////////


std::string toString(const CommonConfig &cfg)
{
    std::stringstream ss;

    ss << "FullStepThresholdSpeed  : " << cfg.fullStepThresholdSpeed << " steps/s" << std::endl;
    ss << "ThermalDriftCoefficient : " << cfg.thermalDriftCoefficient << std::endl;
    ss << "OverCurrentThreshold    : " << cfg.overCurrentThreshold << std::endl;
    ss << "StallThreshold          : " << cfg.stallThreshold << std::endl;
    ss << "StepMode                : " << cfg.stepMode << std::endl;
    ss << "SyncSelect              : " << cfg.syncSelect << std::endl;
    ss << "SyncEnable              : " << (cfg.syncEnable ? "Yes" : "No") << std::endl;
    ss << "ControlMode             : " << (int)cfg.controlMode << std::endl;
    ss << "OscillatorSelect        : " << cfg.oscillatorSelect << std::endl;
    ss << "SwitchConfiguration     : " << cfg.switchConfiguration << std::endl;
    ss << "OverCurrentDetection    : " << cfg.overCurrentDetection << std::endl;
    ss << "AlarmState              : " << std::endl <<  cfg.alarmState << std::endl;
    
    return ss.str();
}

std::string toString(const VoltageModeCfg &backEmfConfig)
{
    std::stringstream ss;
    ss << "KVAL_HOLD  : " << (int) backEmfConfig.holdingKVal	     << std::endl;
    ss << "KVAL_RUN   : " << (int) backEmfConfig.constantSpeedKVal << std::endl;
    ss << "KVAL_ACC   : " << (int) backEmfConfig.accelStartingKVal << std::endl;
    ss << "KVAL_DEC   : " << (int) backEmfConfig.decelStartingKVal << std::endl;
    ss << "INT_SPEED  : " << backEmfConfig.intersectSpeed    << std::endl;
    ss << "ST_SLP     : " << backEmfConfig.startSlope        << std::endl;
    ss << "FN_SLP_ACC : " << backEmfConfig.accelFinalSlope   << std::endl;
    ss << "FN_SLP_DEC : " << backEmfConfig.decelFinalSlope   << std::endl;
    ss << "Slew Rate  : " << backEmfConfig.slewRate << std::endl;
    ss << "VoltageCompensation    : " << backEmfConfig.voltageCompensation << std::endl;
    ss << "PwmFrequencyMultiplier : " << backEmfConfig.pwmFrequencyMultiplier << std::endl;
    ss << "PwmFrequencyDivider    : " << backEmfConfig.pwmFrequencyDivider << std::endl;
    return ss.str();
}

void
VoltageModeCfg::readFromFile(const std::string &file)
{
    // Let's do this in json format (easier to parse)
    pt::ptree root;

    try
    {
        pt::read_json(file,root);

        holdingKVal       = root.get<int>("holdingKVal");
        constantSpeedKVal = root.get<int>("constantSpeedKVal");
        accelStartingKVal = root.get<int>("accelStartingKVal");
        decelStartingKVal = root.get<int>("decelStartingKVal");

        intersectSpeed  = root.get<int>("intersectSpeed");
        startSlope      = root.get<int>("startSlope");
        accelFinalSlope = root.get<int>("accelFinalSlope");
        decelFinalSlope = root.get<int>("decelFinalSlope");
        enableLowSpeedOptimisation = root.get<bool>("enableLowSpeedOptimisation");

        tryReadConfig<SlewRate>(root.get<std::string>("slewRate") ,getSlewRateBiMap(), slewRate);
        tryReadConfig<VoltageCompensation>(root.get<std::string>("voltageCompensation"), getVoltageCompensationBiMap(), voltageCompensation);
        tryReadConfig<PwmFrequencyMultiplier>(root.get<std::string>("pwmFrequencyMultiplier"),getPwmFrequencyMultiplierBiMap(), pwmFrequencyMultiplier);
        tryReadConfig<PwmFrequencyDivider>(root.get<std::string>("pwmFrequencyDivider"),getPwmFrequencyDividerBiMap(), pwmFrequencyDivider);
    }
    catch (std::exception &e)
    {
        std::cout << "Exception thrown while trying to read VoltageModeCfg from file " << file << " with reason " << e.what();
        throw;
    }
}

VoltageModeCfg::VoltageModeCfg():
    holdingKVal(0),
    constantSpeedKVal(0),
    accelStartingKVal(0),
    decelStartingKVal(0),
    intersectSpeed(0),
    startSlope(0),
    accelFinalSlope(0),
    decelFinalSlope(0),
    slewRate(CONFIG_SR_180V_us),
    voltageCompensation(CONFIG_VS_COMP_DISABLE),
    pwmFrequencyMultiplier(CONFIG_PWM_MUL_1),
    pwmFrequencyDivider(CONFIG_PWM_DIV_1),
    enableLowSpeedOptimisation(true)
{    
}

VoltageModeCfg::VoltageModeCfg(const std::string &file):
    VoltageModeCfg()
{
    readFromFile(file);
}

void
VoltageModeCfg::writeToFile(const std::string &cfgFilePath)
{
    pt::ptree root;

    root.put("holdingKVal",holdingKVal);
    root.put("constantSpeedKVal",constantSpeedKVal);
    root.put("accelStartingKVal",accelStartingKVal);
    root.put("decelStartingKVal",decelStartingKVal);

    root.put("intersectSpeed",intersectSpeed);
    root.put("startSlope",startSlope);
    root.put("accelFinalSlope",accelFinalSlope);
    root.put("decelFinalSlope",decelFinalSlope);

    root.put("slewRate",toString(slewRate));
    root.put("voltageCompensation",toString(voltageCompensation));
    root.put("pwmFrequencyMultiplier",toString(pwmFrequencyMultiplier));
    root.put("pwmFrequencyDivider",toString(pwmFrequencyDivider));
    root.put("enableLowSpeedOptimisation",enableLowSpeedOptimisation);

    // Open the file and write to json
    std::ofstream outFile;
    outFile.open(cfgFilePath);
        //throw; // TODO - fix to real exception
    pt::write_json(outFile,root);
}

void
CommonConfig::readFromFile(const std::string &file)
{
    // Let's do this in json format (easier to parse)
    pt::ptree root;

    try
    {
        pt::read_json(file,root);

        fullStepThresholdSpeed  = root.get<int>("fullStepThresholdSpeed");
        thermalDriftCoefficient = root.get<int>("thermalDriftCoefficient");

        tryReadConfig<CurrentThreshold>(root.get<std::string>("overCurrentThreshold"), getCurrentThresholdBiMap(), overCurrentThreshold);
        tryReadConfig<CurrentThreshold>(root.get<std::string>("stallThreshold"), getCurrentThresholdBiMap(),stallThreshold);
        tryReadConfig<StepMode>(root.get<std::string>("stepMode"), getStepModeBiMap(),stepMode);
        tryReadConfig<SyncSelect>(root.get<std::string>("syncSelect"), getSyncSelectBiMap(),syncSelect);
        tryReadConfig<ControlMode>(root.get<std::string>("controlMode"), getControlModeBiMap(),controlMode);
        tryReadConfig<OscillatorSelect>(root.get<std::string>("oscillatorSelect"), getOscillatorSelectBiMap(),oscillatorSelect);
        tryReadConfig<SwitchConfiguration>(root.get<std::string>("switchConfiguration"), getSwitchConfigurationBiMap(), switchConfiguration);
        tryReadConfig<OverCurrentDetection>(root.get<std::string>("overCurrentDetection"), getOverCurrentDetectionBiMap(), overCurrentDetection);

        // Read Sync Enable
        syncEnable = root.get<bool>("syncEnable");

        // Parse Alarm State Config (if it exists)
        pt::ptree alarmRoot = root.get_child("alarmState");
        alarmState.overCurrentEnabled = alarmRoot.get<bool>("overCurrentEnabled");
        alarmState.thermalShutdownEnabled = alarmRoot.get<bool>("thermalShutdownEnabled");
        alarmState.thermalWarningEnabled = alarmRoot.get<bool>("thermalWarningEnabled");
        alarmState.underVoltageEnabled = alarmRoot.get<bool>("underVoltageEnabled");
        alarmState.stallDetectionAEnabled = alarmRoot.get<bool>("stallDetectionAEnabled");
        alarmState.stallDetectionBEnabled = alarmRoot.get<bool>("stallDetectionBEnabled");
        alarmState.switchTurnOnEnabled = alarmRoot.get<bool>("switchTurnOnEnabled");
        alarmState.badCommandEnabled = alarmRoot.get<bool>("badCommandEnabled");
    }
    catch (std::exception &e)
    {
        std::cout << "Exception caught whilst trying to read CommonConfig from file " << file << " with reason " << e.what() << std::endl;
        throw;
    }
}

void
CommonConfig::writeToFile(const std::string &cfgFilePath)
{
    pt::ptree root;

    root.put("fullStepThresholdSpeed",fullStepThresholdSpeed);
    root.put("thermalDriftCoefficient",thermalDriftCoefficient);

    root.put("overCurrentThreshold",toString(overCurrentThreshold));
    root.put("stallThreshold",toString(stallThreshold));

    // STEP_MODE register settings
    root.put("stepMode",toString(stepMode));
    root.put("syncSelect",toString(syncSelect));
    root.put("controlMode",toString(controlMode)); // Voltage or current (NB: some chips will generally support one or the other)
    root.put("syncEnable",syncEnable);

    // CONFIG register settings
    root.put("oscillatorSelect",toString(oscillatorSelect));
    root.put("switchConfiguration",toString(switchConfiguration));
    root.put("overCurrentDetection",toString(overCurrentDetection));

    // Alarm Register Settings
    pt::ptree alarmStateNode;
    alarmStateNode.put("overCurrentEnabled",alarmState.overCurrentEnabled);
    alarmStateNode.put("thermalShutdownEnabled",alarmState.thermalShutdownEnabled);
    alarmStateNode.put("thermalWarningEnabled",alarmState.thermalWarningEnabled);
    alarmStateNode.put("underVoltageEnabled",alarmState.underVoltageEnabled);
    alarmStateNode.put("stallDetectionAEnabled",alarmState.stallDetectionAEnabled);
    alarmStateNode.put("stallDetectionBEnabled",alarmState.stallDetectionBEnabled);
    alarmStateNode.put("switchTurnOnEnabled",alarmState.switchTurnOnEnabled);
    alarmStateNode.put("badCommandEnabled",alarmState.badCommandEnabled);
    root.add_child("alarmState",alarmStateNode);

    // Open the file and write to json
    std::ofstream outFile;
    outFile.open(cfgFilePath);
        //throw; // TODO - fix to real exception
    pt::write_json(outFile,root);
}

void
CurrentModeCfg::readFromFile(const std::string &file)
{
    assert("!TODO");
}

void
CurrentModeCfg::writeToFile(const std::string &cfgFilePath)
{
    assert(!"TODO - commoncfg writing");
}
