#include "simdriver.h"

SimDriver::SimDriver(const std::vector<StepperMotor> &motors):
    motors_(motors)
{}

// Contains all data in status command
// and also the current position and speed
std::vector<Status> SimDriver::getStatus()
{
    std::vector<Status> statusVector= {Status()};
    return statusVector;
}
Status
SimDriver::getStatus(int motor){
    // TODO
    return Status();
}

std::vector<Status>
SimDriver::clearStatus()
{
    return getStatus();
}

// Individual get functions if only very specific data needed
std::vector<bool>
SimDriver::isBusy(){
    std::vector<bool> isBusyVector = {false};
    return isBusyVector;
}

bool
SimDriver::isBusy(int motor){
    return false;
}

std::vector<int32_t>
SimDriver::getPos(){
    std::vector <int32_t> positions= {0};
    return positions;
}

int32_t
SimDriver::getPos(int motor){
    return 0;
}

std::vector<uint32_t>
SimDriver::getSpeed(){
    std::vector <uint32_t> positions= {0};
    return positions;
} // steps/s

uint32_t
SimDriver::getSpeed(int motor){
    return 0;
}

std::vector<int32_t>
SimDriver::getMark(){
    std::vector <int32_t> positions= {0};
    return positions;
}  // steps?

int32_t
SimDriver::getMark(int motor){
    return 0;
}

////////////////////////////////////////
/// Profile Configuration Commands
////////////////////////////////////////

void SimDriver::setConfig(AbstractConfig &config , int motor){
}

// Profile is different, this we want to set efficiently since it is real-time critical
void SimDriver::setProfileCfg(const std::map<int,ProfileCfg> &cfgs){
}

void SimDriver::setProfileCfg(const ProfileCfg &cfg , int motor){

}

//Profile Raw parameters
void SimDriver::setAcc(std::map<int,float> &accelerations){

}

void SimDriver::setAcc(float stepsPerSecondPerSecond , int motor ){

}

void SimDriver::setDec(std::map<int,float> &decelerations){

}

void SimDriver::setDec(float stepsPerSecondPerSecond , int motor ){

}

void SimDriver::setMaxSpeed(const std::map <int,float> &maxSpeeds){

}

void SimDriver::setMaxSpeed(float stepsPerSecond , int motor){

}

void SimDriver::setMinSpeed(const std::map <int,float> &minSpeeds){
}

void SimDriver::setMinSpeed(float stepsPerSecond , int motor){
}

ProfileCfg
SimDriver::getProfileCfg(int motor){
    return ProfileCfg();
}

// void setFullSpeed(float stepsPerSecond){ }

//////////////////////////////
/// Operational Commands
/////////////////////////////

// Speed Commands
void SimDriver::run(const std::map<int, DataCommand> &runCommands){

}

void SimDriver::run(const RunCommand &runCommand , int motor){
}

void SimDriver::goUntil(const std::map<int, DataCommand> &goUntilCommands){

}

void SimDriver::goUntil(const GoUntilCommand &command , int motor){

}

//    void releaseSw(const std::map <int,ReleaseSwCommand> &releaseSWCommands){ }
//    void releaseSw(const ReleaseSwCommand &command , int motor){ }

// Position Commands
void SimDriver::move(const std::map <int,DataCommand> &moveCommands){

}

void SimDriver::move(const MoveCommand &command , int motor){

}

void SimDriver::goTo(const std::map <int,DataCommand> &goToCommands){

}

void SimDriver::goTo(const GoToCommand &command , int motor){

}

void SimDriver::goToDir(const std::map <int,DataCommand> &goToDirCommands){
}

void SimDriver::goToDir(const GoToDirCommand &command , int motor){
}

void SimDriver::goHome(const std::vector <int> &motors){

}
void SimDriver::goHome(int motor){

}

void SimDriver::goMark(const std::vector <int> &motors){

}

void SimDriver::goMark(int motor){

}

// Stop Commands
void SimDriver::softStop(const std::vector <int> &motors){

}

void SimDriver::softStop(int motor){

}

void SimDriver::hardStop(const std::vector <int> &motors){

}

void SimDriver::hardStop(int motor){

}

void SimDriver::softHiZ(const std::vector <int> &motors){

}

void SimDriver::softHiZ(int motor){

}

void SimDriver::hardHiZ(const std::vector <int> &motors){

}

void SimDriver::hardHiZ(int motor){

}

// Set Commands
//void setMark(const std::map<int, long> &marks){ }
//void setPos(const std::map<int,long> &newPositions){ }
void SimDriver::setPos  (int32_t pos , int motor){

}

void SimDriver::resetPos(const std::vector <int> &motors){

}

void SimDriver::resetPos(int motor){

}

void SimDriver::resetDev(const std::vector <int> &motors){

}
