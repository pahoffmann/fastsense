#include "fpga_program.h"

#include <iostream>
#include <example/vadd.h>

using namespace fastsense::hw;

FPGAProgram::FPGAProgram(const char* xclbin_filename) 
    :   nb_{0},
        buf_{new char[nb_]},
        devices_{},
        binfile_{},
        context_{},
        program_{}
{
    std::cout << "Initializing Devices\n";
    initDevices();
    std::cout << "Init context\n";
    initContext();
    std::cout << "Loading: '" << xclbin_filename << "'\n";
    loadXCLBIN(xclbin_filename);
    std::cout << "Creating Program from Binary File\n";
    createProgramFromBinaryFile();
}

const cl::Device& FPGAProgram::getDevice() const 
{
    if (not devices_.size())
    {
        throw std::runtime_error("Error: Unable to init Context. No devices found");
    }
    return devices_[0];
}

cl::Context& FPGAProgram::getContext()
{
    return context_;
}

const cl::Program& FPGAProgram::getProgram() const
{
    return program_;
}

void FPGAProgram::initDevices() 
{
    std::cout << "Initializing Devices\n";
    std::vector<cl::Platform> platforms;

    //traversing all Platforms To find Xilinx Platform and targeted
    //Device in Xilinx Platform
    cl::Platform::get(&platforms);
    for (size_t i = 0; (i < platforms.size() & not devices_.size()); i++)
    {
        cl::Platform platform = platforms[i];
        std::string platformName = platform.getInfo<CL_PLATFORM_NAME>();
        if (platformName == "Xilinx")
        {
            devices_.clear();
            platform.getDevices(CL_DEVICE_TYPE_ACCELERATOR, &devices_);
            if (devices_.size())
            {
                return;
            }
        }
    }

    throw std::runtime_error("Error: Unable to find Target Device");
}

void FPGAProgram::initContext()
{
    if (not devices_.size())
    {
        throw std::runtime_error("Error: Unable to init Context. No devices found");
    }

    context_ = cl::Context(devices_[0]);
}

void FPGAProgram::loadXCLBIN(const char* xclbin_filename) 
{
    binfile_ = std::ifstream{xclbin_filename, std::ifstream::binary};
    binfile_.seekg (0, binfile_.end);
    nb_ = binfile_.tellg();
    binfile_.seekg(0, binfile_.beg);
    binfile_.read(buf_, nb_);
}

void FPGAProgram::createProgramFromBinaryFile()
{
    bins_.push_back({buf_, nb_});
    devices_.resize(1);
    program_ = cl::Program{context_, devices_, bins_};
}