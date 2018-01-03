#include "commands.h"
#include <sstream>
#include <assert.h>
#include "support.h"

uint32_t capMaxValue(uint32_t value , int bitLength)
{
    uint32_t mask = 0xffffffff >> (32-bitLength);
    return value&mask;
}

uint32_t toTwosComplementUint(int32_t value , int bitLength)
{
    uint32_t result = 0;
    result = ((~value)+1) & (0xFFFFFFFF >> (32-bitLength));
    return result;
}

int32_t toSignedInt(uint32_t value , int bitLength)
{
    if (value& (0x01 << bitLength))
    {
        return -((uint32_t)(~value+1)&(0xFFFFFFFF >> (32-bitLength)));
    }
    return value;
}

std::string toString (const std::map<int,DataCommand> &commandMap)
{
    std::stringstream ss;

    for (const auto &x : commandMap)
    {
        ss << "Motor " << x.first <<" --> [ cmd = " << x.second.cmd << " , flags = " << x.second.cmdFlags << " , data = " << x.second.data << " , bitLength = " << x.second.dataBitLength << "]" << std::endl;
    }
    return ss.str();
}

std::string toString(const RunCommand &x)
{
    std::stringstream ss;
    ss <<"direction = " << x.direction << " , speed = " << x.stepsPerSec << "steps/s" << std::endl;
    return ss.str();
}

std::string toString(const GoUntilCommand &x)
{
    std::stringstream ss;
    ss <<"direction = " << x.direction << " , speed = " << x.stepsPerSec << "steps/s" << " , Action = " << x.action;
    return ss.str();
}

std::string toString(const MoveCommand &x)
{
    std::stringstream ss;
    ss << "direction = " << x.direction << " , steps = " << x.numSteps << " steps";
    return ss.str();
}

std::string toString(const GoToCommand &command)
{
    return "pos = " + command.pos;
}

std::string toString(const GoToDirCommand &x)
{
    std::stringstream ss;
    ss << "direction = " << x.direction << " , position = " << x.pos;
    return ss.str();
}

int toBitLength(char paramRegister)
{
    int retVal=0;

    switch (paramRegister)
    {

    // ABS_POS is the current absolute offset from home. It is a 22 bit number expressed
    //  in two's complement. At power up, this value is 0. It cannot be written when
    //  the motor is running, but at any other time, it can be updated to change the
    //  interpreted position of the motor.

    // MARK is a second position other than 0 that the motor can be told to go to. As
    //  with ABS_POS, it is 22-bit two's complement. Value is 0 on power up.

    case ABS_POS:
    case MARK:
        retVal = 22;
        break;

    // SPEED contains information about the current speed. It is read-only. It does
    //  NOT provide direction information.
    case SPEED:
        retVal = 20;
        break;

    // CONFIG contains some assorted configuration bits and fields. A fairly comprehensive
    //  set of reasonably self-explanatory constants is provided, but users should refer
    //  to the datasheet before modifying the contents of this register to be certain they
    //  understand the implications of their modifications. Value on boot is 0x2E88; this
    //  can be a useful way to verify proper start up and operation of the dSPIN chip.

    // STATUS contains read-only information about the current condition of the chip. A
    //  comprehensive set of constants for masking and testing this register is provided, but
    //  users should refer to the datasheet to ensure that they fully understand each one of
    //  the bits in the register.

    case CONFIG:
    case STATUS:  // STATUS is a read-only register
        retVal = 16;
        break;

    // INT_SPD, ST_SLP, FN_SLP_ACC and FN_SLP_DEC are all related to the back EMF
    //  compensation functionality. Please see the datasheet for details of this
    //  function- it is too complex to discuss here. Default values seem to work
    //  well enough.
    case INT_SPD:
        retVal = 14;
        break;

    case MIN_SPEED:
        retVal = 13;
        break;

    case ACC:
    case DECEL:
        retVal = 12;
        break;

    case MAX_SPEED:
    case FS_SPD:
        retVal = 10;
        break;

    case EL_POS:
        retVal = 9;
        break;

    case KVAL_HOLD:
    case KVAL_RUN:
    case KVAL_ACC:
    case KVAL_DEC:
    case ST_SLP:
    case FN_SLP_ACC:
    case FN_SLP_DEC:
    case K_THERM:
    case ADC_OUT:
    case OCD_TH:
    case STALL_TH:
    case STEP_MODE:
    case ALARM_EN:
        retVal = 8;
        break;
    }

    return retVal;
}
