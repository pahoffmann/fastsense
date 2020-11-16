#pragma once

/**
 * @file registration_input.h
 * @author Julian Gaal
 */

#include <util/point.h>
#include <msg/point_cloud.h>
#include <msg/zmq_converter.h>

namespace fastsense::msg
{

class RegistrationInput : public ZMQConverter
{
public:
    RegistrationInput(const Matrix4f& global_transform, PointCloud pcl)
    : global_transform_{global_transform}, pcl_{std::move(pcl)}
    {}

    ~RegistrationInput() = default;

    void from_zmq_msg(zmq::multipart_t& msg) override
    {
        global_transform_ = msg.poptyp<Matrix4f>();
        pcl_.from_zmq_msg(msg);
    }

    zmq::multipart_t to_zmq_msg() const override
    {
        zmq::multipart_t multi;
        multi.addtyp(global_transform_);
        multi.append(pcl_.to_zmq_msg());
        return multi;
    }
private:
    Matrix4f global_transform_;
    PointCloud pcl_;
};

}