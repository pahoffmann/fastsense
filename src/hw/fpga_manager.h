/**
 * @file fpga_manager.h
 * @author Julian Gaal, Marcel Flottmann
 * @date 2020-09-10
 */

#pragma once

#include <hw/opencl.h>
#include <hw/types.h>
#include <fstream>

namespace fastsense::hw
{

class FPGAManager
{
public:
    ~FPGAManager() = default;

    FPGAManager(const FPGAManager&) = delete;
    FPGAManager& operator=(const FPGAManager&) = delete;

    static void loadXCLBIN(const std::string& xclbin_filename);

    static const cl::Device& getDevice();
    static const cl::Context& getContext();
    static const cl::Program& getProgram();

    static CommandQueuePtr createCommandQueue();

private:
    FPGAManager();

    static FPGAManager& inst();

    std::vector<cl::Device> devices_;
    cl::Context context_;
    cl::Program program_;

    void initDevices();
    void initContext();
    void loadProgram(const std::string& xclbin_filename);
};

} // namespace fastsense::hw