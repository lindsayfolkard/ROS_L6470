#include "basedriver.h"
#include "commands.h"
#include <mraa.hpp>
#include <exception>
#include <vector>

std::vector<int> getAllMotorsVector(int numMotors)
{
    std::vector<int> motors;

    for (int i=0; i < numMotors ; ++i)
    {
        motors.push_back(i);
    }

    return motors;
}

BaseDriver::BaseDriver(const std::vector<StepperMotor> &motors, MotorDriverType motorDriverType, int spiBus, CommsDebugLevel commsDebugLevel):
    motors_(motors),
    motorDriverType_(motorDriverType),
    commsDebugLevel_(commsDebugLevel)
{
    commsDriver_.reset(new CommsDriver(motors_.size(),spiBus,commsDebugLevel));
}

void
BaseDriver::checkMotorIsValid(int motor)
{
    if (motor < 0 || (unsigned int)motor > motors_.size() )
        throw WrongMotorException("Motor number " + std::to_string(motor) + " is invalid (expect num < " + std::to_string((int)motors_.size()) + ")");
}

/////////////////////////
/// Status Commands
/////////////////////////

// Contains all data in status command
// and also the current position and speed
std::vector<Status>
BaseDriver::getStatus()
{
    if (commsDebugLevel_ >= CommsDebugOnlyActions) 
    {
	std::cout << "++++++++++++++++ (CommsDebug) : Get Status ++++++++++++++++++++++" << std::endl << std::endl;
    }

    // Get the responses
    //std::map<int,uint32_t> emptyMap;
    std::vector<uint32_t> states = commsDriver_->getParam(STATUS,toBitLength(STATUS));

    // Parse the responses
    std::vector<Status> statusVector;
    for (auto state : states)
    {
        statusVector.push_back(parseStatus(state));
    }

    if (commsDebugLevel_ >= CommsDebugOnlyActions) 
    {
	std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl << std::endl;
    }

    return statusVector;
}

Status
BaseDriver::getStatus(int motor)
{
    checkMotorIsValid(motor);
    std::vector<Status> states = getStatus();
    return states[motor];
}

std::vector<Status>
BaseDriver::clearStatus()
{
    if (commsDebugLevel_ >= CommsDebugOnlyActions)
    {
    std::cout << "++++++++++++++++ (CommsDebug) : Get Status ++++++++++++++++++++++" << std::endl << std::endl;
    }

    // Send the request
    commsDriver_->SPIXfer(GET_STATUS);

    // Get the responses
    std::map<int,uint32_t> emptyMap;
    std::vector<uint32_t> states = commsDriver_->SPIXfer(emptyMap,toBitLength(STATUS));

    // Parse the responses
    std::vector<Status> statusVector;
    for (auto state : states)
    {
        statusVector.push_back(parseStatus(state));
    }

    if (commsDebugLevel_ >= CommsDebugOnlyActions)
    {
    std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl << std::endl;
    }

    return statusVector;
}

// Individual get functions if only very specific data needed
std::vector<bool>
BaseDriver::isBusy()
{
    std::vector<Status> statusVector =  getStatus();
    std::vector<bool> isBusyVector = {false};
    for (unsigned int i=0; i < statusVector.size() ; ++i)
    {
        isBusyVector[i] = statusVector[i].isBusy;
    }
    return isBusyVector;
}

bool
BaseDriver::isBusy(int motor)
{
    checkMotorIsValid(motor);
    std::vector<bool> busyVector = isBusy();

    return busyVector[motor];
}

std::vector<int32_t> BaseDriver::getPos()
{
    std::vector <uint32_t> positions = commsDriver_->getParam(ABS_POS);
    std::vector <int32_t> convertedPositions;
    for (auto element : positions)
    {
        // Since ABS_POS is a 22-bit 2's comp value, we need to check bit 21 and, if
        //  it's set, set all the bits ABOVE 21 in order for the value to maintain
        //  its appropriate sign.
        //if (element & 0x00200000) element |= 0xffc00000;
        // hack
        //convertedPositions.push_back(element);
        //std::cout << "Debug - element ins position vector is : " << (int) element << std::endl;
        convertedPositions.push_back(toSignedInt(element,toBitLength(ABS_POS)));
    }
    return convertedPositions;
}

int32_t BaseDriver::getPos(int motor)
{
    checkMotorIsValid(motor);
    return getPos()[motor];
}

std::vector<uint32_t>
BaseDriver::getSpeed()
{
    std::vector <uint32_t> speeds = commsDriver_->getParam(SPEED);

    for (auto &element : speeds)
    {
        element = spdParse(element);
    }
    return speeds;
}

uint32_t
BaseDriver::getSpeed(int motor)
{
    checkMotorIsValid(motor);
    std::vector<uint32_t> speeds = getSpeed();
    return speeds[motor];
}

std::vector<int32_t> BaseDriver::getMark()
{
    std::vector <uint32_t> marks = commsDriver_->getParam(MARK);
    std::vector <int32_t> convertedMarks;
    for (auto element : marks)
    {
        // Since ABS_POS is a 22-bit 2's comp value, we need to check bit 21 and, if
        //  it's set, set all the bits ABOVE 21 in order for the value to maintain
        //  its appropriate sign.
        //if (element & 0x00200000) element |= 0xffC00000;
        convertedMarks.push_back(toSignedInt(element,toBitLength(MARK)));
    }
    return convertedMarks;
}

int32_t BaseDriver::getMark(int motor)
{
    checkMotorIsValid(motor);
    std::vector<int32_t> marks = getMark();
    return marks[motor];
}

void
BaseDriver::setProfileCfg(const std::map<int,ProfileCfg> &cfgs)
{
    std::map<int,float> accel;
    std::map<int,float> decel;
    std::map<int,float> maxVel;
    std::map<int,float> minVel;

    for (const auto motor : cfgs)
    {
        accel.insert(std::pair<int,float>(motor.first,motor.second.acceleration));
        decel.insert(std::pair<int,float>(motor.first,motor.second.deceleration));
        maxVel.insert(std::pair<int,float>(motor.first,motor.second.maxSpeed));
        minVel.insert(std::pair<int,float>(motor.first,motor.second.minSpeed));
    }

    setAcc(accel);
    setDec(decel);
    setMaxSpeed(maxVel);
    setMinSpeed(minVel);
}

void
BaseDriver::setProfileCfg(const ProfileCfg &cfg, int motor)
{
    checkMotorIsValid(motor);
    std::map<int,ProfileCfg> cfgMap = {{motor,cfg}};
    setProfileCfg(cfgMap);
}

ProfileCfg
BaseDriver::getProfileCfg(int motor)
{
    ProfileCfg profileCfg;
    profileCfg.acceleration = getAcc(motor);
    profileCfg.deceleration = getDec(motor);
    profileCfg.maxSpeed     = getMaxSpeed(motor);
    profileCfg.minSpeed     = getMinSpeed(motor);

    return profileCfg;
}

// Set the acceleration rate, in steps per second per second. This value is
//  converted to a dSPIN friendly value. Any value larger than 29802 will
//  disable acceleration, putting the chip in "infinite" acceleration mode.
void
BaseDriver::setAcc(float stepsPerSecondPerSecond , int motor)
{
  uint32_t integerAcc = accCalc(stepsPerSecondPerSecond);
  commsDriver_->setParam(ACC, toBitLength(ACC), integerAcc, motor);
}

void
BaseDriver::setAcc(std::map<int,float> &accelerations)
{
    std::map<int,uint32_t> intAccelerations;

    for (const auto element : accelerations)
    {
        intAccelerations.insert(std::pair<int,uint32_t>(element.first,accCalc(element.second)));
    }
    commsDriver_->setParam(ACC, toBitLength(ACC),intAccelerations);
}

float
BaseDriver::getAcc(int motor)
{
  return accParse(commsDriver_->getParam(ACC, toBitLength(ACC), motor));
}

// Same rules as setAcc().
void
BaseDriver::setDec(float stepsPerSecondPerSecond, int motor)
{
  uint32_t integerDec = decCalc(stepsPerSecondPerSecond);
  commsDriver_->setParam(DECEL, toBitLength(DECEL), integerDec, motor);
}

void
BaseDriver::setDec(std::map<int,float> &decelerations)
{
    std::map<int,uint32_t> intDecelerations;

    for (const auto element : decelerations)
    {
        intDecelerations.insert(std::pair<int,uint32_t>(element.first,decCalc(element.second)));
    }
    commsDriver_->setParam(DECEL, toBitLength(DECEL),intDecelerations);
}

float
BaseDriver::getDec(int motor)
{
  return accParse(commsDriver_->getParam(DECEL, toBitLength(DECEL), motor));
}

void
BaseDriver::setMaxSpeed(const std::map <int,float> &maxSpeeds)
{
    std::map<int,uint32_t> intMaxSpeeds;
    for (const auto element : maxSpeeds)
    {
        intMaxSpeeds.insert(std::pair<int,uint32_t>(element.first,maxSpdCalc(element.second)));
    }

    commsDriver_->setParam(MAX_SPEED,toBitLength(MAX_SPEED),intMaxSpeeds);
}

// This is the maximum speed the dSPIN will attempt to produce.
void
BaseDriver::setMaxSpeed(float stepsPerSecond , int motor)
{
  // We need to convert the floating point stepsPerSecond into a value that
  //  the dSPIN can understand. Fortunately, we have a function to do that.
 uint32_t integerSpeed = maxSpdCalc(stepsPerSecond);

  // Now, we can set that paramter.
  commsDriver_->setParam(MAX_SPEED, toBitLength(MAX_SPEED), integerSpeed, motor);
}

void
BaseDriver::setMinSpeed(const std::map <int,float> &minSpeeds)
{
    std::map<int,uint32_t> intMinSpeeds;
    for (const auto element : minSpeeds)
    {
        // MIN_SPEED also contains the LSPD_OPT flag, so we need to protect that.
        uint32_t temp = commsDriver_->getParam(MIN_SPEED, toBitLength(MIN_SPEED), element.first) & 0x00001000;

        intMinSpeeds.insert(std::pair<int,uint32_t>(element.first,minSpdCalc(element.second)|temp));
    }

    commsDriver_->setParam(MIN_SPEED, toBitLength(MIN_SPEED), intMinSpeeds);
}

// Set the minimum speed allowable in the system. This is the speed a motion
//  starts with; it will then ramp up to the designated speed or the max
//  speed, using the acceleration profile.
void BaseDriver::setMinSpeed(float stepsPerSecond, int motor)
{
  // We need to convert the floating point stepsPerSecond into a value that
  //  the dSPIN can understand. Fortunately, we have a function to do that.
  uint32_t integerSpeed = minSpdCalc(stepsPerSecond);

  // MIN_SPEED also contains the LSPD_OPT flag, so we need to protect that.
  uint32_t temp = commsDriver_->getParam(MIN_SPEED, toBitLength(MIN_SPEED), motor) & 0x00001000;

  // Now, we can set that paramter.
  commsDriver_->setParam(MIN_SPEED, toBitLength(MIN_SPEED), integerSpeed | temp , motor);
}

float BaseDriver::getMaxSpeed(int motor)
{
  return maxSpdParse(commsDriver_->getParam(MAX_SPEED, toBitLength(MAX_SPEED), motor));
}

float BaseDriver::getMinSpeed(int motor)
{
  return minSpdParse(commsDriver_->getParam(MIN_SPEED, toBitLength(MIN_SPEED), motor));
}


/////////////////////////
/// Operational Commands
////////////////////////

// Speed Commands

//  RUN sets the motor spinning in a direction (defined by the constants
//  FWD and REV). Maximum speed and minimum speed are defined
//  by the MAX_SPEED and MIN_SPEED registers; exceeding the FS_SPD value
//  will switch the device into full-step mode.
//  The spdCalc() function is provided to convert steps/s values into
//  appropriate integer values for this function.
void
BaseDriver::run(const std::map<int, RunCommand> &runCommands)
{
    commsDriver_->sendCommands<RunCommand>(runCommands);
}

void
BaseDriver::run(const RunCommand &runCommand , int motor)
{
    commsDriver_->sendCommand(runCommand,motor);
}

// GoUntil will set the motor running with direction dir (REV or
//  FWD) until a falling edge is detected on the SW pin. Depending
//  on bit SW_MODE in CONFIG, either a hard stop or a soft stop is
//  performed at the falling edge, and depending on the value of
//  act (either RESET or COPY) the value in the ABS_POS register is
//  either RESET to 0 or COPY-ed into the MARK register.
void
BaseDriver::goUntil(const std::map<int, GoUntilCommand> &goUntilCommands)
{
    commsDriver_->sendCommands<GoUntilCommand>(goUntilCommands);
}

void
BaseDriver::goUntil(const GoUntilCommand &command, int motor)
{
    commsDriver_->sendCommand(command,motor);
}

// Similar in nature to GoUntil, ReleaseSW produces motion at the
//  higher of two speeds: the value in MIN_SPEED or 5 steps/s.
//  The motor continues to run at this speed until a rising edge
//  is detected on the switch input, then a hard stop is performed
//  and the ABS_POS register is either COPY-ed into MARK or RESET to
//  0, depending on whether RESET or COPY was passed to the function
//  for act.

/// TODO !
//void
//BaseDriver::releaseSw(const std::map<int, ReleaseSwCommand> &releaseSWCommands)
//{
//    commsDriver_->commsDriver_->sendCommands(releaseSWCommands);
//}

//void
//BaseDriver::releaseSw(const ReleaseSwCommand &command , int motor)
//{
//    commsDriver_->sendCommand(command,motor);
//}

// Position Commands
void
BaseDriver::move(const std::map<int, MoveCommand> &moveCommands)
{
    commsDriver_->sendCommands<MoveCommand>(moveCommands);
}

void
BaseDriver::move(const MoveCommand &command , int motor)
{
    commsDriver_->sendCommand(command,motor);
}

void
BaseDriver::goTo(const std::map<int, GoToCommand> &goToCommands)
{
    commsDriver_->sendCommands<GoToCommand>(goToCommands);
}

void
BaseDriver::goTo(const GoToCommand &command , int motor)
{
    commsDriver_->sendCommand(command,motor);
}

void
BaseDriver::goToDir(const std::map<int, GoToDirCommand> &goToDirCommands)
{
    commsDriver_->sendCommands<GoToDirCommand>(goToDirCommands);
}

void
BaseDriver::goToDir(const GoToDirCommand &command , int motor)
{
    commsDriver_->sendCommand(command,motor);
}

void
BaseDriver::goHome(const std::vector<int> &motors)
{
    commsDriver_->sendCommands(motors,GO_HOME);
}

void
BaseDriver::goHome(int motor)
{
    goHome(std::vector<int>{motor});
}

void
BaseDriver::goMark(const std::vector<int> &motors)
{
    commsDriver_->sendCommands(motors,GO_MARK);
}

void
BaseDriver::goMark(int motor)
{
    goMark(std::vector<int>{motor});
}

// Set Commands
void
BaseDriver::setMark(int32_t pos, int motor)
{
    commsDriver_->setParam(MARK,toBitLength(MARK),toTwosComplementUint(pos,toBitLength(MARK)), motor);
}

//void
//BaseDriver::setPos(const std::map<int,long> &newPositions)
//{
//    setParam(ABS_POS,newPositions);
//}

void
BaseDriver::setPos(int32_t pos , int motor)
{
   commsDriver_->setParam(ABS_POS ,toBitLength(ABS_POS), toTwosComplementUint(pos,toBitLength(ABS_POS)) , motor);
}

void
BaseDriver::setAllPos(int32_t pos)
{
    for (unsigned int i =0; i < motors_.size(); ++i)
    {
        std::cout << "Debug - position set for motor " << i << " = " << pos << std::endl;
        setPos(pos,i);
        std::cout << "Debug - position read back for motor " << i << " = " << getPos(i) << std::endl;
    }
}

void
BaseDriver::resetPos(const std::vector<int> &motors)
{
    commsDriver_->sendCommands(motors,RESET_POS);
}

void
BaseDriver::resetPos(int motor)
{
    checkMotorIsValid(motor);
    std::vector<int> motors = {motor};
    resetPos(motors);
}

void
BaseDriver::resetDev(const std::vector<int> &motors)
{
    commsDriver_->sendCommands(motors,RESET_DEVICE);
}

// Stop Commands
void
BaseDriver::stopAllHard()
{
    hardStop(getAllMotorsVector(motors_.size()));
}

void
BaseDriver::stopAllSoft()
{
    softStop(getAllMotorsVector(motors_.size()));
}


void
BaseDriver::softStop(const std::vector<int> &motors)
{
    commsDriver_->sendCommands(motors,SOFT_STOP);
}

void
BaseDriver::softStop(int motor)
{
    checkMotorIsValid(motor);
    softStop(std::vector<int>{motor});
}

void
BaseDriver::hardStop(const std::vector<int> &motors)
{
    commsDriver_->sendCommands(motors,HARD_STOP);
}

void
BaseDriver::hardStop(int motor)
{
    checkMotorIsValid(motor);
    hardStop(std::vector<int>{motor});
}

void
BaseDriver::softHiZ(const std::vector<int> &motors)
{
    commsDriver_->sendCommands(motors,SOFT_HIZ);
}

void
BaseDriver::softHiZ(int motor)
{
    checkMotorIsValid(motor);
    softHiZ(std::vector<int>{motor});
}

void
BaseDriver::hardHiZ(const std::vector<int> &motors)
{
    commsDriver_->sendCommands(motors,HARD_HIZ);
}

void
BaseDriver::hardHiZ(int motor)
{
    checkMotorIsValid(motor);
    hardHiZ(std::vector<int> {motor});
}
