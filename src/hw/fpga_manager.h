/**
 * @file fpga_manager.h
 * @author Julian Gaal, Marcel Flottmann
 * @date 2020-09-10
 */

#pragma once

#include <hw/opencl.h>
#include <hw/buffer/buffer.h>
#include <hw/types.h>
#include <fstream>

namespace fastsense::hw
{

class FPGAManager
{
public:
    explicit FPGAManager(const char* xclbin_filename);
    ~FPGAManager() = default;

    FPGAManager(const FPGAManager&) = delete;
    FPGAManager& operator=(const FPGAManager&) = delete;

    const cl::Device& getDevice() const;
    const cl::Context& getContext();
    const cl::Program& getProgram() const;

    CommandQueuePtr createCommandQueue();

private:
    std::vector<cl::Device> devices_;
    cl::Context context_;
    cl::Program program_;

    void initDevices();
    void initContext();
    void loadProgram(const char* xclbin_filename);
};

} // namespace fastsense::hw