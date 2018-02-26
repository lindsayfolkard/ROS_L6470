#include <iostream>
#include "../abstractdriver.h"
#include "../powerstepdriver.h"
#include "../motor.h"
#include <stdexcept>
#include <string>
#include <regex>
#include <assert.h>
#include <chrono>

using namespace std;

int main(int argc, char **argv)
{
    // Create the stepper motor
    Stepper_42BYGHW811 stepper;
    std::vector<StepperMotor> motors = {stepper};

    PowerStepDriver driver(motors);

    driver.softHiZ(0);

    while(1)
    {
      std::cout << "Status is : " << std::endl << driver.getStatus(0) << std::endl;
      std::cout << "Status after clearing is :" << std::endl << driver.clearStatus()[0] << std::endl;
      usleep(500000);
    }
}
