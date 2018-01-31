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

int main (int argc, char ** argv)
{
    // Create the stepper motor
    Stepper_42BYGHW811 stepper;
    std::vector<StepperMotor> motors = {stepper};

    // Let's try to make a PowerStepConfig from motors
    VoltageModeCfg voltageModeCfg = BackEmfConfigFromStepper(stepper);
    CurrentModeCfg currentModeCfg;
    CommonConfig   commonConfig;
    PowerStepCfg powerStepConfig(commonConfig,currentModeCfg,voltageModeCfg);
    std::vector<PowerStepCfg> cfgs = {powerStepConfig};
    // Instantiate the AutoDriver
    cout << "Try to instantiate the driver" << endl;
    PowerStepDriver driver(motors,cfgs,0,CommsDebugOnlyActions);
    cout << "Instantiated the driver!" << endl;

    // Lets try and get the status
    usleep(1000);
    cout << "Status is " << driver.getStatus(0) << endl;

    // Get the position
    usleep(1000);
    cout << "Initial pos = " << driver.getPos(0) << endl;

    // Get the Speed
    usleep(1000);
    cout << "Initial speed = " << driver.getSpeed(0) << endl;

    // Lets zero the position
    cout << "Set position to 0";
    usleep(1000);
    driver.setPos(0,0);

    // Get the position
    usleep(1000);
    cout << "Position = " << driver.getPos(0);

    // Let's try to go to a position
    GoToCommand goToCommand(10000);
    driver.goTo(goToCommand);
    while (1)
    {
        cout << "Motor position is : " << driver.getPos() << endl;
        usleep(5000);
    }

    //    // Read the config
    //    cout << " Profile config" <<  driver.getProfileCfg(0) << endl;

    //    ProfileCfg profile;
    //    profile.acceleration=100;
    //    profile.deceleration=120;
    //    profile.maxSpeed=140;
    //    profile.minSpeed=160;

    //    cout << "Set new profule config : " << profile << endl;
    //    driver.setProfileCfg(profile,0);

    //    cout << " New profile config is : " << driver.getProfileCfg(0) << endl;
    //    cout << "Status is : " << driver.getStatus()[0] << endl;

    //    cout << "Clear the status : " << driver.clearStatus()[0] << endl;
    //    // Enable drive
    //    cout << "New status is : " << driver.getStatus(0) << endl;

    //    cout << "||||||||||||||||||||||||||||||||||||||" << endl;
    //    cout << "Lets read some simple shit : " << endl;
    //    cout << "Pos = " << driver.getPos(0)<< "steps" << endl;
    //    cout << "Speed = " << driver.getSpeed(0) << " steps/s" << endl;
    //    //cout << " = " << driver.g

    //    // Let's try to read a config
    //    PowerStepCfg cfg = driver.getConfig(0);
    //    cout << "Config : " << endl << cfg << endl;

    //    cout << " Set a cfg with a few different values" << endl;
    //    cfg.commonCfg_.alarmState.switchTurnOnEnabled=false;
    //    cfg.commonCfg_.alarmState.underVoltageEnabled=false;
    //    cfg.commonCfg_.controlMode=VoltageControlMode;
    //    cfg.commonCfg_.oscillatorSelect=CONFIG_INT_16MHZ_OSCOUT_2MHZ;
    //    cfg.commonCfg_.fullStepThresholdSpeed=500;
    //    cfg.commonCfg_.stallThreshold=OCD_TH_6375m;
    //    cout << "Desired new config " << cfg << endl;

    //    cout << "Set the config" << endl;

    //    driver.setConfig(cfg,0);

    //    cout << "Config is set" << endl;
    //    cout << "Reread the config " << endl;
    //    cout << "New Config is " << driver.getConfig(0) << endl;

//    // Let's try to do some simple shit
//    const auto start = std::chrono::steady_clock::now();
//    std::vector<Status> statusVector = driver.getStatus();
//    const auto end = std::chrono::steady_clock::now();
//    std::chrono::duration<double> diff = end-start;
//    int count=1;
//    for (const Status &status : statusVector)
//    {
//	cout << "Status for motor" << count++ << " is " << status << std::endl << std::endl;
//    }
    
//    cout << "Test setting the motors to softStop" << std::endl;
//    cout << "Status : " << driver.getStatus(0) << " , time = " << diff.count()*1000000 << " microseconds" <<  std::endl;
//    driver.softStop(0);
//    driver.softStop(1);
//    driver.softStop(2);
//    cout << "Status : " << driver.getStatus(0) << std::endl;

//    usleep(3e6);

//    driver.softStop({0,1,2});

//    // Lets get position
//    cout << "Position : " << driver.getPos(0) << std::endl;
//    cout << "Config   : " << driver.getConfig(0) << std::endl;
//    driver.resetPos(0);
//    cout << "Position : " << driver.getPos(0) << std::endl;

//    // Lets set the config
//    cout << "Original Config is " << driver.getConfig(0);
//    Stepper_57BYGH51 nema23BackEmfConfig;
//    VoltageModeCfg backEmfConfig = BackEmfConfigFromStepper(nema23BackEmfConfig,24,2.5);
//    Config driverConfig;
//    driverConfig.backEmfConfig = backEmfConfig;
//    driverConfig.overCurrentThreshold = OCD_TH_3750m;
//    driver.setConfig(driverConfig,0);
//    cout << "Updated Config is " << driver.getConfig(0);
    
//    // Lets set a profile config
//    ProfileCfg profileCfg = driver.getProfileCfg(0);
//    cout << "ProfileCfg" << profileCfg << std::endl;
//    profileCfg.maxSpeed=650;
//    profileCfg.minSpeed=80;
//    profileCfg.acceleration=100;
//    profileCfg.deceleration=100;

//    driver.setProfileCfg(profileCfg,0);
//    cout << "New profile cfg is " << profileCfg << std::endl;

    // Fuck it let's try to move
    /*driver.softStop(0);
    RunCommand runCommand(Forward,400);
    driver.run(runCommand,0);

    const int timeThreshold = 6;
    double tLive = 0.0;
    while (tLive < timeThreshold)
    {
      const auto end = std::chrono::steady_clock::now();
      std::chrono::duration<double> diff = end-start;
      tLive = diff.count();
      cout << "............ T + " << tLive << " s ..............." << std::endl;
      cout << "Position = " << driver.getPos(0) << std::endl;
      cout << "Speed = " << driver.getSpeed(0) << std::endl;
      cout << "Status = " << driver.getStatus(0) << std::endl;
      std::chrono::duration<double> diff2 = std::chrono::steady_clock::now()-end;
      cout << "Data Acquisition time : " << diff2.count()*1000.0 << " ms" << std::endl;
      cout << "......................................" << std::endl << std::endl;

      usleep(20000);

      if (tLive > 3.0)
      {
         driver.hardStop(0);
	 //driver.softHiZ(0);
      }
    }

    driver.softStop(0);
    */
    //    // Read the config back

    //    printMenu();

    //    // Enter Command Loop
    //    while (true)
    //    {
    //        cout <<">>";
    //        std::string input;
    //        cin >> input;
    //        try
    //        {
    //            handleMenuOption(input,driver);
    //        }
    //        catch (std::exception &e)
    //        {
    //            cout << "Caught exception with message " << e.what() << endl;
    //            cout << "Continuing in the hope everything is fine"<<endl;
    //        }
    //    }
}
