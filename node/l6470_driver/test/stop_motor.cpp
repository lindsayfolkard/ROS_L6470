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
    (void) argc;
    (void) argv;

    if (argc < 2)
    {
        std::cout << "Insufficient Arguments. Usage " << argv[0] << " numMotors" << std::endl;
        return EXIT_FAILURE;
    }

    const int numMotors = std::stoi(argv[1]);

    // Create the stepper motor
    Stepper_42BYGHW811 stepper;
    std::vector<StepperMotor> motors(numMotors,stepper);

    PowerStepDriver driver(motors);

    driver.stopAllHard();

    for (int i=0; i < numMotors; ++i)
    {
        std::cout << "==================================================================" << std::endl;
        std::cout << "Motor " << i << " status is : " << std::endl << driver.getStatus(0) << std::endl;
        std::cout << "Motor " << i << " status after clearing is :" << std::endl << driver.clearStatus()[0] << std::endl;
        std::cout << "==================================================================" << std::endl;
        usleep(1000);
    }
}
