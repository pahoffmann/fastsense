#pragma once

#include <CL/cl2.hpp>

namespace fastsense::kernels {

// TODO saver setArg(s)

class BaseKernel : public cl::Kernel
{
private:
    int narg_;
protected:
    template <typename ...T>
    inline void setArgs(const T ...args) {
        std::array<cl::Buffer, sizeof...(ArgsT)> const buffers {args...};

        for (const auto& bf: buffers) {
            setArg(narg_++, bf);
        }
    }

    template <typename T>
    inline void setArg(const T arg) override {
        setArg(narg_++, arg);
    }

    template <typename T>
    inline void setArg(int narg, const T arg) {
        setArg(narg, arg);
    }

    // T* getVirtual

    cl::Kernel kernel_;   
public:
    inline BaseKernel(const cl::Program& program, char* name) : kernel_(program, name), narg_(0) {}
    virtual ~BaseKernel() = default;
};

} // namespace fastsense::kernels