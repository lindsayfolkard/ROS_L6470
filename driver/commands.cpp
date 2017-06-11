#include "commands.h"

///
/// Run Commands
///

std::string toString(const RunCommand &x)
{
    std::stringstream ss;
    ss <<"direction = " << x.direction << " , speed = " << x.stepsPerSec << "steps/s" << std::endl;
    return ss.str();
}

uint8_t
RunCommand::toCommand()
{
    return (RUN | direction);
}

long
RunCommand::toData(int &bitLength)
{
    bitLength=toBitLength(SPEED);
    return toBigEndian(capMaxValue(spdCalc(stepsPerSec),bitLength));
}

///
/// GoUntilCommand
///
uint8_t
GoUntilCommand::toCommand()
{
    return (GO_UNTIL | action | direction);
}

long
GoUntilCommand::toData(int &bitLength)
{
    bitLength = toBitLength(SPEED);
    return toBigEndian(capMaxValue(spdCalc(stepsPerSec),bitLength));
}

std::string toString(const GoUntilCommand &x)
{
    std::stringstream ss;
    ss <<"direction = " << x.direction << " , speed = " << x.stepsPerSec << "steps/s" << " , Action = " << x.action;
    return ss.str();
}

///
/// MoveCommand
///

uint8_t
MoveCommand::toCommand()
{
    return (MOVE|direction);
}

long
MoveCommand::toData(int &bitLength)
{
    bitLength = toBitLength(ABS_POS);
    return toBigEndian(capMaxValue(numSteps,bitLength));
}

std::string toString(const MoveCommand &x)
{
    std::stringstream ss;
    ss << "direction = " << x.direction << " , steps = " << x.numSteps << " steps";
    return ss.str();
}

///
/// GoToCommand
///

std::String toString(const GoToCommand &command)
{
    return "pos = " << command.pos;
}

uint8_t
GoToCommand::toCommand()
{
    return GOTO;
}

long
GoToCommand::toData()
{
    return capMaxValue(pos,toBitLEngth(ABS_POS));
}

///
/// GoToDirCommand
///

uint8_t
GoToDirCommand::toCommand()
{
    return (GOTO_DIR | direction);
}

long
GoToDirCommand::toData(int &bitLength)
{
    bitLength = toBitLength(ABS_POS);
    return toBigEndian(capMaxValue(pos,bitLength));
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
