#include "powerstepdriver.h"

PowerStepDriver::PowerStepDriver(const std::vector<StepperMotor> &motors, int spiBus, CommsDebugLevel commsDebugLevel):
    BaseDriver(motors,spiBus,commsDebugLevel)
{

}

