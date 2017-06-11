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
    MotorSpinDirection direction;
    float stepsPerSec;

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
    MotorSpinDirection direction;
    float stepsPerSec;
    Action action;

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

// Making this a struct so that it makes sense w.r.t everything else...
struct GoToCommand : public DataCommand
{
    long pos;
    uint8_t toCommand() override;
    long toData(int &bitLength) override;
};

struct GoToDirCommand : public DataCommand
{
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
