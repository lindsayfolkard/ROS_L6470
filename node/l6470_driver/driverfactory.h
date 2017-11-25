#pragma once

#include "config.h"
#include "abstractdriver.h"
#include <memory>

///
/// \brief The DriverFactory class
/// \author Lindsay Folkard
/// \abstract A simple class to handle construction of the correct driver based on the input config files (i.e instantiates powerstep01,l6470 or l6472 driver)

std::unique_ptr<AbstractDriver> factoryMakeDriver(const OverallCfg &overallCfg);
