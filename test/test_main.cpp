/**
 * @file test_main.cpp
 * @author Marcel Flottmann
 * @date 2020-09-14
 */

#define CATCH_CONFIG_RUNNER
#include "catch2_config.h"

#include <hw/fpga_manager.h>
#include <util/logging/logger.h>

using namespace fastsense::util::logging;
using namespace fastsense::hw;

int main( int argc, char* argv[] )
{
    auto logSink = std::make_shared<sink::FileSink>("test.log");
    Logger::addSink(logSink);

    FPGAManager::loadXCLBIN("FastSense.xclbin");

    int result = Catch::Session().run(argc, argv);

    return result;
}
