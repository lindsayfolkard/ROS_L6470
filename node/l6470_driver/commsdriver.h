#pragma once

#include <vector>
#include <map>
#include <mraa.hpp>
#include <memory>
#include <stdint.h>
#include <assert.h>
#include "commands.h"

enum CommsDebugLevel
{
  CommsDebugNothing=0,
  CommsDebugOnlyActions=1,
  CommsDebugEverything=2
};

// CommsDriver
// Description : A class to handle all communications to and from
//               daisy chained stspin controllers (L647*,L648*,PowerStep01)

class CommsDriver
{
public:

    CommsDriver(int numMotors , int spiBus = 0);

    // Sends requestValue to all motors
    std::vector<uint32_t> SPIXfer(uint8_t requestValue);

    // Send requestValue to only the specified motor
    uint32_t SPIXfer(uint8_t requestValue, int motor);

    std::vector<uint32_t> SPIXfer(const std::map<int, uint32_t> &data , int bitLength);
    std::vector<uint32_t> xferParam(const std::map<int,uint32_t> &parameters, uint8_t bitLen);

    // Convenience commands to sendCommands
    void sendCommands(const std::map <int,DataCommand> &dataCommands);
    void sendCommands(const std::vector<int> &motors , uint8_t commandByte);
    void sendCommand (const DataCommand &dataCommand , int motor);

    // Set Parameter Commands
    void setParam(uint8_t paramRegister, uint8_t bitLength, std::map<int, uint32_t> &values);
    void setParam(uint8_t paramRegister, uint8_t bitLength, uint32_t value , int motor);
    std::vector<uint32_t> getParam(uint8_t paramRegister, uint8_t bitLength);
    std::vector<uint32_t> getParam(uint8_t paramRegister);
    uint32_t getParam(uint8_t paramRegister, uint8_t bitLength, int motor);

private:

    void checkMotorIsValid(int motor);

    // Template validity check
    template <typename T> void checkMapIsValid (const std::map <int,T> &input)
    {
        if (input.size() > (unsigned int) numMotors_)
        {
            assert (!"Invalid map size");
        }

        for (const auto &element : input)
        {
            if (element.first >= numMotors_ || element.first < 0)
            {
                assert(!"Invalid map entry");
            }
        }
    }

    CommsDebugLevel            commsDebugLevel_;
    std::unique_ptr<mraa::Spi> SPI_;
    const int                  numMotors_;
};
