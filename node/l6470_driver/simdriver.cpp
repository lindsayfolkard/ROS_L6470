#include "simdriver.h"

SimDriver::SimDriver(const std::vector<StepperMotor> &motors):
    motors_(motors)
{}

SimDriver::SimDriver(const std::vector<StepperMotor> &motors,                                 std::vector<PowerStepCfg> cfgs):
    motors_(motors),
    cfgs_(cfgs)
{
    std::cout << "========= Motor Configs ==========" << std::endl;
    int count=0;
    for (const auto &motor : motors_)
    {
        std::cout << "Motor " << count << std::endl << motor;
        ++count;
    }
    std::cout << "==================================" << std::endl;

    std::cout << "======= Power Step Configs =======" << std::endl;
    count=0;
    for (const auto &cfg : cfgs_)
    {
        std::cout << "Motor " << count << std::endl << cfg;
        ++count;
    }
    std::cout << "==================================" << std::endl;

}

// Contains all data in status command
// and also the current position and speed
std::vector<Status> SimDriver::getStatus()
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    Status defaultStatus = Status();
    std::vector<Status> statusVector(motors_.size(),defaultStatus);
    return statusVector;
}
Status
SimDriver::getStatus(int motor){
    std::cout << "SimDriver : " << __func__  << std::endl;
    return Status();
}

std::vector<Status>
SimDriver::clearStatus()
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    return getStatus();
}

// Individual get functions if only very specific data needed
std::vector<bool>
SimDriver::isBusy()
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    std::vector<bool> isBusyVector(motors_.size());
    return isBusyVector;
}

bool
SimDriver::isBusy(int motor)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    return false;
}

std::vector<int32_t>
SimDriver::getPos()
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    std::vector <int32_t> positions(motors_.size(),0);
    return positions;
}

int32_t
SimDriver::getPos(int motor)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    return 0;
}

std::vector<uint32_t>
SimDriver::getSpeed(){
    std::vector <uint32_t>  speeds(motors_.size(),0);
    return speeds;
} // steps/s

uint32_t
SimDriver::getSpeed(int motor){
    return 0;
}

std::vector<int32_t>
SimDriver::getMark(){
    std::cout << "SimDriver : " << __func__  << std::endl;
    std::vector <int32_t> positions(motors_.size(),0);
    return positions;
}  

int32_t
SimDriver::getMark(int motor)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    return 0;
}

////////////////////////////////////////
/// Profile Configuration Commands
////////////////////////////////////////

void
SimDriver::setConfig(AbstractConfig &config , int motor)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // do nothing
}

void
SimDriver::setProfileCfg(const std::map<int,ProfileCfg> &cfgs)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    profileCfgs_ = cfgs;
}

void
SimDriver::setProfileCfg(const ProfileCfg &cfg , int motor)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    profileCfgs_.insert(std::pair<int,ProfileCfg>(motor,cfg));
    // TODO - change speed of motors if moving
}

//Profile Raw parameters
void
SimDriver::setAcc(std::map<int,float> &accelerations)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void SimDriver::setAcc(float stepsPerSecondPerSecond , int motor )
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    std::map<int,float> accelerations;
    accelerations.insert(std::pair<int,float>(motor,stepsPerSecondPerSecond));
    setAcc(accelerations);
}

void
SimDriver::setDec(std::map<int,float> &decelerations)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::setDec(float stepsPerSecondPerSecond , int motor )
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    std::map<int,float> decelMap;
    decelMap.insert(std::pair<int,float>(motor,stepsPerSecondPerSecond));
    setDec(decelMap);
}

void
SimDriver::setMaxSpeed(const std::map <int,float> &maxSpeeds)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::setMaxSpeed(float stepsPerSecond , int motor)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::setMinSpeed(const std::map <int,float> &minSpeeds)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void SimDriver::setMinSpeed(float stepsPerSecond , int motor)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

ProfileCfg
SimDriver::getProfileCfg(int motor)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    return ProfileCfg();
}

// void setFullSpeed(float stepsPerSecond){ }

//////////////////////////////
/// Operational Commands
/////////////////////////////

// Speed Commands
void
SimDriver::run(const std::map<int, RunCommand> &runCommands)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::run(const RunCommand &runCommand , int motor)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::goUntil(const std::map<int, GoUntilCommand> &goUntilCommands)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::goUntil(const GoUntilCommand &command , int motor)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

//    void releaseSw(const std::map <int,ReleaseSwCommand> &releaseSWCommands){ }
//    void releaseSw(const ReleaseSwCommand &command , int motor){ }

// Position Commands
void
SimDriver::move(const std::map<int, MoveCommand> &moveCommands)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::move(const MoveCommand &command , int motor)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::goTo(const std::map<int, GoToCommand> &goToCommands)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::goTo(const GoToCommand &command , int motor)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::goToDir(const std::map<int, GoToDirCommand> &goToDirCommands)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::goToDir(const GoToDirCommand &command , int motor)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::goHome(const std::vector <int> &motors)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}
void
SimDriver::goHome(int motor)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::goMark(const std::vector <int> &motors)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::goMark(int motor)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

// Stop Commands

void
SimDriver::stopAllHard()
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::stopAllSoft()
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::softStop(const std::vector <int> &motors)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::softStop(int motor)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::hardStop(const std::vector <int> &motors)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::hardStop(int motor)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::softHiZ(const std::vector <int> &motors)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::softHiZ(int motor)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::hardHiZ(const std::vector <int> &motors)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::hardHiZ(int motor)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

// Set Commands
//void setMark(const std::map<int, long> &marks){ }
//void setPos(const std::map<int,long> &newPositions){ }
void
SimDriver::setPos  (int32_t pos , int motor)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::setMark(int32_t pos, int motor)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
    // do nothing
}

void
SimDriver::setAllPos(int32_t pos)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO -
}

void
SimDriver::resetPos(const std::vector <int> &motors)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::resetPos(int motor)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::resetDev(const std::vector <int> &motors)
{
    std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}
