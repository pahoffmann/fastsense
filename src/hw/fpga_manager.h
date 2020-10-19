#pragma once

/**
 * @file fpga_manager.h
 * @author Julian Gaal
 * @author Marcel Flottmann
 */

#include <fstream>

#include <hw/opencl.h>

namespace fastsense::hw
{

class FPGAManager
{
public:
    ~FPGAManager() = default;

    FPGAManager(const FPGAManager&) = delete;
    FPGAManager& operator=(const FPGAManager&) = delete;

    static void load_xclbin(const std::string& xclbin_filename);

    static const cl::Device& get_device();
    static const cl::Context& get_context();
    static const cl::Program& get_program();

    static CommandQueuePtr create_command_queue();

    static void release();

private:
    FPGAManager();

    static std::unique_ptr<FPGAManager>& inst();

    std::vector<cl::Device> devices_;
    cl::Context context_;
    cl::Program program_;

    void init_devices();
    void init_context();
    void load_program(const std::string& xclbin_filename);
};

} // namespace fastsense::hw