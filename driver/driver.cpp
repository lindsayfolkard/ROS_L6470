#include "driver.h"
#include <mraa.hpp>

int AutoDriver::_numBoards;

// Constructors
AutoDriver::AutoDriver(int position, int CSPin, int resetPin, int busyPin)
{
  _CSPin = CSPin;
  _position = position;
  _resetPin = resetPin;
  _busyPin = busyPin;
  _numBoards++;

  // Try to initialise the mraa::SPI port
  _SPI.reset(new mraa::Spi(0));
  _SPI->mode(mraa::SPI_MODE3);
  _SPI->frequency(4000000); // Can this be a bit higher ?
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
