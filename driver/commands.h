#pragma once

#include "types.h"
#include <iostream>
#include <string.h>
#include "support.h"

///
/// Contains definitions of commands sent to and from the DSPIN L6470H
/// We define as structs to standardise the sending of commands
/// (Given the fact that we will typically have lists of commands and
/// different commands require different flags)
///

// Conversion Functions
uint32_t toBigEndian (uint32_t value);
uint32_t capMaxValue(uint32_t value , int bitLength);
int toBitLength(ParamRegister paramRegister);

struct DataCommand
{
    DataCommand(Command _cmd   , uint8_t _cmdFlags ,
                uint32_t _data , uint8_t _dataBitLength):
        cmd(_cmd),
        cmdFlags(_cmdFlags),
        data(_data),
        dataBitLength(_dataBitLength)
    {}

    // Command Byte to send
    const Command  cmd;
    const uint8_t  cmdFlags;
    uint8_t toCommand() const { return ((uint8_t)cmd | cmdFlags);}

    // DataByte to send
    const uint32_t data;
    const uint8_t  dataBitLength;
};

struct RunCommand : public DataCommand
{
    RunCommand(MotorSpinDirection _direction , float _stepsPerSec):
        DataCommand(
            RUN,
            _direction,
            toBigEndian(capMaxValue(spdCalc(_stepsPerSec),toBitLength(SPEED))),
            toBitLength(SPEED)
        ),
        direction(_direction),
        stepsPerSec(_stepsPerSec){}

    // Variables
    const MotorSpinDirection direction;
    const float stepsPerSec;
};

std::string toString(const RunCommand &x);
inline std::ostream& operator<<(std::ostream& os,const RunCommand &x)
{
    return os << toString(x);
}

enum Action
{
    Action_Reset_AbsPos = 0x00,
    Action_Copy_AbsPos  = 0x08
};

struct GoUntilCommand : public DataCommand
{
    // Constructor
    GoUntilCommand(MotorSpinDirection _direction , float _stepsPerSec , Action _action):
        DataCommand(
            GO_UNTIL,
            direction|_action,
            toBigEndian(capMaxValue(spdCalc(stepsPerSec),toBitLength(SPEED))),
            toBitLength(SPEED)
        ),
        direction(_direction),
        stepsPerSec(_stepsPerSec),
        action(_action)
        {}

    // Variables
    MotorSpinDirection direction;
    float stepsPerSec;
    Action action;
};

std::string toString(const GoUntilCommand &x);
inline std::ostream& operator<<(std::ostream& os,const GoUntilCommand &x)
{
    return os << toString(x);
}

struct MoveCommand : public DataCommand
{
    MoveCommand(MotorSpinDirection _direction , unsigned long _numSteps):
        DataCommand(
            MOVE,
            _direction,
            toBigEndian(capMaxValue(numSteps,toBitLength(ABS_POS))),
            toBitLength(ABS_POS)
        ),
        direction(_direction),
        numSteps(_numSteps){}

    MotorSpinDirection direction;
    unsigned long numSteps;
};

std::string toString(const MoveCommand &x);
inline std::ostream& operator<<(std::ostream& os,const MoveCommand &x)
{
    return os << toString(x);
}

// Making this a struct so that it makes sense w.r.t everything else sending a command
struct GoToCommand : public DataCommand
{
    GoToCommand(long _pos):
        DataCommand(
            GOTO,
            0x00,
            toBigEndian(capMaxValue(pos,toBitLength(ABS_POS))),
            toBitLength(ABS_POS)
        ),
        pos(_pos){}

    long pos;
};

struct GoToDirCommand : public DataCommand
{
    GoToDirCommand(MotorSpinDirection _direction , long _pos):
        DataCommand(
            GOTO_DIR,
            _direction,
            toBigEndian(capMaxValue(pos,toBitLength(ABS_POS))),
            toBitLength(ABS_POS)
        ),
        direction(_direction),
        pos(_pos)
        {}

    MotorSpinDirection direction;
    long pos;
};

std::string toString(const GoToDirCommand &x);
inline std::ostream& operator<<(std::ostream& os,const GoToDirCommand &x)
{
    return os << toString(x);
}
