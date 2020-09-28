/**
 * @file fpga_manager.cpp
 * @author Julian Gaal, Marcel Flottmann
 * @date 2020-09-10
 */

#include "fpga_manager.h"

#include <iostream>
#include <util/logging/logger.h>

using namespace fastsense::hw;
using namespace fastsense::util::logging;

FPGAManager::FPGAManager()
    : devices_{},
      context_{},
      program_{}
{
}

void FPGAManager::loadXCLBIN(const std::string& xclbin_filename)
{
    inst().initDevices();
    inst().initContext();
    inst().loadProgram(xclbin_filename);
}

FPGAManager& FPGAManager::inst()
{
    static FPGAManager manager;
    return manager;
}

const cl::Device& FPGAManager::getDevice()
{
    if (not inst().devices_.size())
    {
        throw std::runtime_error("Error: Unable to init Context. No devices found");
    }
    return inst().devices_[0];
}

const cl::Context& FPGAManager::getContext()
{
    return inst().context_;
}

const cl::Program& FPGAManager::getProgram()
{
    return inst().program_;
}

fastsense::CommandQueuePtr FPGAManager::createCommandQueue()
{
    return std::make_shared<cl::CommandQueue>(getContext(), getDevice(), CL_QUEUE_OUT_OF_ORDER_EXEC_MODE_ENABLE);
}

void FPGAManager::initDevices()
{
    Logger::info("Initializing Device");
    std::vector<cl::Platform> platforms;

    //traversing all Platforms To find Xilinx Platform and targeted
    //Device in Xilinx Platform
    cl::Platform::get(&platforms);
    Logger::info("Found ", platforms.size(), " platforms");
    for (size_t i = 0; i < platforms.size(); i++)
    {
        cl::Platform& platform = platforms[i];
        std::string platformName = platform.getInfo<CL_PLATFORM_NAME>();
        Logger::info("Platform ", i, ": ", platformName);
        if (platformName == "Xilinx")
        {
            devices_.clear();
            platform.getDevices(CL_DEVICE_TYPE_ACCELERATOR, &devices_);
            Logger::info("Platform ", i, ": Found ", devices_.size(), " devices");
            if (devices_.size())
            {
                Logger::info("Device found");
                return;
            }
        }
    }

    throw std::runtime_error("Error: Unable to find Target Device");
}

void FPGAManager::initContext()
{
    if (not devices_.size())
    {
        throw std::runtime_error("Error: Unable to init Context. No devices found");
    }

    context_ = cl::Context(devices_);
}

void FPGAManager::loadProgram(const std::string& xclbin_filename)
{
    Logger::info("Loading ", xclbin_filename);

    // read binary file content
    std::ifstream bin_file{xclbin_filename, std::ifstream::binary};
    bin_file.seekg (0, bin_file.end);
    unsigned nb = bin_file.tellg();
    bin_file.seekg (0, bin_file.beg);
    char* buf = new char [nb];
    bin_file.read(buf, nb);

    // create program from binary
    cl::Program::Binaries binaries;
    binaries.emplace_back(buf, nb);
    program_ = cl::Program{context_, devices_, binaries};

    delete[] buf;
}