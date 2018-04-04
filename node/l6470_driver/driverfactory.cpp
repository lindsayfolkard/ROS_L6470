#include "driverfactory.h"
#include <assert.h>

#include "powerstepdriver.h"
#include "l6470driver.h"
#include "l6472driver.h"
#include "simdriver.h"

std::unique_ptr<AbstractDriver> factoryMakeDriver(const OverallCfg &overallCfg)
{
    // Read the required stepper motor config and driver config files
    std::vector<StepperMotor>   motors;
    std::vector<PowerStepCfg> cfgs;

    for (const auto &cfgFile : overallCfg.cfgFiles_)
    {
        motors.push_back(StepperMotor(cfgFile));

        switch (overallCfg.controllerType_)
        {
        case PowerStep01:
            cfgs.push_back(PowerStepCfg(cfgFile));
            break;
        case L6470:
            // TODO!
            break;
        case L6472:
            // TODO!
            break;
        case Simulator:
            cfgs.push_back(PowerStepCfg(cfgFile));
            break;
        default:
            assert(!"Invalid controller type in driverfactory.cpp factoryMakeDriver");
        }
    }

    // Instantiate the correct driver
    std::unique_ptr<AbstractDriver> driver;

    switch (overallCfg.controllerType_)
    {
    case PowerStep01:
        driver.reset(new PowerStepDriver(motors,cfgs,overallCfg.spiBus_,overallCfg.commsDebugLevel_));
        break;
    case L6470:
        // TODO!
        assert(!"TODO! - L6470 driver not yet implemented in driverfactory.cpp");
        break;
    case L6472:
        // TODO!
        assert(!"TODO! - L6472 driver not yet implemented in driverfactory.cpp");
        break;
    case Simulator:
        driver.reset(new SimDriver(motors,cfgs));
        break;
    default:
        assert(!"Invalid controller type in driverfactory.cpp factoryMakeDriver");
    }

    return std::move(driver);
}
