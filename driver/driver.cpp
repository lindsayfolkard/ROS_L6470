#include "driver.h"

int AutoDriver::_numBoards;

// Constructors
AutoDriver::AutoDriver(int position, int CSPin, int resetPin, int busyPin)
{
  _CSPin = CSPin;
  _position = position;
  _resetPin = resetPin;
  _busyPin = busyPin;
  _numBoards++;
  //_SPI = &SPI;
}

AutoDriver::AutoDriver(int position, int CSPin, int resetPin)
{
  _CSPin = CSPin;
  _position = position;
  _resetPin = resetPin;
  _busyPin = -1;
  _numBoards++;
  //_SPI = &SPI;
}

void
AutoDriver::SPIPortConnect(std::unique_ptr<mraa::Spi> SPIPort)
{
  _SPI = std::move(SPIPort);
}

bool
AutoDriver::isBusy(void)
{
  if (_busyPin == -1)
  {
    if (getParam(STATUS) & 0x0002) return false;
    else                           return true;
  }
  return false; // TODO - correct this action
//  else
//  {
//    if (digitalRead(_busyPin) == HIGH) return 0;
//    else                               return 1;
//  }
}
