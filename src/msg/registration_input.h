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
    RegistrationInput()
    : acc_transform_{Matrix4f::Identity()}, pcl_{}
    {}

    RegistrationInput(const Matrix4f& acc_transform, PointCloud pcl)
    : acc_transform_{acc_transform}, pcl_{std::move(pcl)}
    {}

    ~RegistrationInput() override = default;

    void from_zmq_msg(zmq::multipart_t& msg) override
    {
        zmq::message_t acc_transform_msg = msg.pop();
        memcpy(&acc_transform_, acc_transform_msg.data(), sizeof(Matrix4f));
        pcl_.from_zmq_msg(msg);
    }

    zmq::multipart_t to_zmq_msg() const override
    {
        zmq::multipart_t multi;
        multi.addtyp(acc_transform_);
        multi.append(pcl_.to_zmq_msg());
        return multi;
    }

    const Matrix4f& acc_transform() const
    {
        return acc_transform_;
    }

    const PointCloud& pcl() const
    {
        return pcl_;
    }

private:
    Matrix4f acc_transform_;
    PointCloud pcl_;
};

}