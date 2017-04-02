#include <mraa.hpp>
#include <memory>

int main(int argc, char **argv)
{

    std::unique_ptr<mraa::Spi> spi;
    spi.reset(new mraa::Spi(0));

}

