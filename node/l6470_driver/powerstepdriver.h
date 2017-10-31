#pragma once

#include "basedriver.h"


class PowerStepDriver : public BaseDriver
{
public:
    PowerStepDriver(const std::vector<StepperMotor> &motors, int spiBus = 0, CommsDebugLevel commsDebugLevel = CommsDebugNothing);
    //void setConfig(const std::vector<> &cfgs);
private:

};
