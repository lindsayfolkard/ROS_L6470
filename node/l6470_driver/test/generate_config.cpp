#include "../config.h"
#include "../motor.h"

int main(int argc, char ** argv)
{
    Stepper_42BYGHW811 nema17Stepper;
    Stepper_57BYGH51   nema23SmallStepper;

    // Let's write the stepper motors to file

    // Let's make the common configs
    CommonConfig   commonCfg;
    commonConfig.overCurrentThreshold = OCD_TH_3375m;
    VoltageModeCfg voltageMode17Cfg = BackEmfConfigFromStepper(nema17Stepper);
    //CurrentModeCfg currentModeCfg;
    // Let's make a bunch of config files

    // Let's make a
    OverallCfg overallCfg;
    overallCfg.commsDebugLevel_ = CommsDebugNothing;
    overallCfg.controllerType_  = PowerStep01;
    overallCfg.spiBus_ = 0;
    overallCfg.cfgFiles_.push_back();
    overallCfg.cfgFiles_.push_back(voltageMode17Cfg);


    // Let's write the motor to file

    // Let's just write the configs to file for the given motor
    commonCfg.writeToFile();

}
