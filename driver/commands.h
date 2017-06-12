#pragma once

#include "types.h"
#include <iostream>
#include <string.h>

///
/// Contains definitions of commands sent to and from the DSPIN L6470H
///

struct DataCommand
{
    // Most commands to the DSPIN contain an initial command byte and then a corresponding series of data bytes
    // Generic function to return the uint8_t command byte (b1 to send to device)
    virtual uint8_t toCommand() = 0;

    // Generic function to return the actual data to send through to the board and the number of bytes to send
    virtual long toData(int &bitLength) = 0;

};

struct RunCommand : public DataCommand
{
    RunCommand(MotorSpinDirection _direction , float _stepsPerSec):
        direction(_direction),
        stepsPerSec(_stepsPerSec){}

    // Variables
    MotorSpinDirection direction;
    float stepsPerSec;

    // Functions
    uint8_t toCommand() override;
    long toData(int &bitLength) override;

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
        direction(_direction),
        stepsPerSec(_stepsPerSec),
        action(_action){}

    // Variables
    MotorSpinDirection direction;
    float stepsPerSec;
    Action action;

    // Functions
    uint8_t toCommand() override;
    long toData(int &bitLength) override;
};
std::string toString(const GoUntilCommand &x);
inline std::ostream& operator<<(std::ostream& os,const GoUntilCommand &x)
{
    return os << toString(x);
}

struct MoveCommand : public DataCommand
{
    MoveCommand(MotorSpinDirection _direction , unsigned long _numSteps):
        direction(_direction),
        numSteps(_numSteps){}

    MotorSpinDirection direction;
    unsigned long numSteps;

    uint8_t toCommand() override;
    long toData(int &bitLength) override;
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
        pos(_pos){}

    long pos;

    uint8_t toCommand() override;
    long toData(int &bitLength) override;
};

struct GoToDirCommand : public DataCommand
{
    GoToDirCommand(MotorSpinDirection _direction , long _pos):
        direction(_direction),
        pos(_pos){}

    MotorSpinDirection direction;
    long pos;

    uint8_t toCommand() override;
    long toData(int &bitLength) override;
};
std::string toString(const GoToDirCommand &x);
inline std::ostream& operator<<(std::ostream& os,const GoToDirCommand &x)
{
    return os << toString(x);
}
