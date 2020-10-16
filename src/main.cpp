/**
 * @file main.cpp
 * @author Employees Washroom
 */

#include <application.h>
#include <util/logging/logger.h>
#include <util/config/config_manager.h>
#include <hw/fpga_manager.h>

using namespace fastsense::util::logging;
using namespace fastsense::util::logging::sink;
using namespace fastsense::util::config;
using namespace fastsense::hw;

int main()
{
    // Initialze Logger
    auto coutSink = std::make_shared<CoutSink>();
    auto fileSink = std::make_shared<FileSink>("FastSense.log");
    Logger::addSink(coutSink);
    Logger::setLoglevel(LogLevel::Debug);

    // Initialize Config
    try
    {
        Logger::info("Load configuration...");
        ConfigManager::loadFile("config.json");
        Logger::info("Configuration loaded");
    }
    catch (const std::exception& e)
    {
        Logger::fatal("Cannot load configuration: ", e.what());
        return -1;
    }

    try
    {
        FPGAManager::load_xclbin("FastSense.xclbin");
    }
    catch (const std::exception& e)
    {
        Logger::fatal("Cannot load XCLBIN: ", e.what());
        return -1;
    }


    int ret = 0;
    try
    {
        // Run Application
        fastsense::Application app;
        ret = app.run();
    }
    catch (const std::exception& e)
    {
        Logger::fatal("Run Application failed: ", e.what());
        return -1;
    }

    FPGAManager::release();

    return ret;
}