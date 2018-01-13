#include "../commsdriver.h"
#include "../config.h"

int main(int argc, char **argv)
{
    // Instantiate a base level comms driver to check that comms are fine
    CommsDriver commsDriver(1,0,CommsDebugNothing);

    // Run the Common Config unit test
    CommonConfig cfg;
    cfg.unitTest(commsDriver,0);
}
