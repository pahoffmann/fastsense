#pragma once

#include <CL/cl2.hpp>
#include <fstream>

namespace fastsense::hw {

class FPGAProgram: public cl::Program 
{
public:
    FPGAProgram(const char* xclbin_filename);
    ~FPGAProgram() = default;

    const cl::Device& getDevice() const;
    cl::Context& getContext();
    const cl::Program& getProgram() const;

private:
    unsigned int nb_;
    char* buf_;
    std::vector<cl::Device> devices_;
    std::ifstream binfile_;
    cl::Program::Binaries bins_;
    cl::Context context_;
    cl::Program program_;

    void initDevices();
    void initContext();
    void loadXCLBIN(const char* xclbin_filename);
    void createProgramFromBinaryFile();
};

} // namespace fastsense::hw