#include "commands.h"
#include <sstream>
#include <assert.h>
#include "support.h"

uint32_t toBigEndian (uint32_t value)
{
    assert("TODO!!");
    return 0;
}

uint32_t capMaxValue(uint32_t value , int bitLength)
{
    assert("TODO!!");
    return 0;
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

int toBitLength(ParamRegister paramRegister)
{
    int retVal=0;

    switch (paramRegister)
    {

    case ABS_POS:
    case MARK:
        retVal = 22;
        break;

    case SPEED:
        retVal = 20;
        break;

    case CONFIG:
    case STATUS:  // STATUS is a read-only register
        retVal = 16;
        break;

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
