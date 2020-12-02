#pragma once

/**
 * @file stamped.h
 * @author Julian Gaal
 */

#include <util/time.h>
#include <msg/zmq_converter.h>

namespace fastsense::msg {

template<typename DATA_T>
struct Stamped : public ZMQConverter
{
    Stamped()
    : data_{}
    , timestamp_{util::HighResTime::now()}
    {
    }

    explicit Stamped(DATA_T data, util::HighResTimePoint timepoint = util::HighResTime::now())
    : data_(std::move(data))
    , timestamp_(timepoint)
    {
    }

    virtual ~Stamped() = default;

    void from_zmq_msg(zmq::multipart_t &msg) override
    {
        convert_from_zmq(msg);
    }

    zmq::multipart_t to_zmq_msg() const override
    {
        return convert_to_zmq();
    }
    
    template < typename TT = DATA_T, std::enable_if_t < std::is_base_of_v<msg::ZMQConverter, TT>, int > = 0 >
    void convert_from_zmq(zmq::multipart_t &msg) 
    {
        timestamp_ = msg.poptyp<util::HighResTimePoint>();
        data_.from_zmq_msg(msg);
    }

    template < typename TT = DATA_T, std::enable_if_t < !std::is_base_of_v<msg::ZMQConverter, TT>, int > = 0 >
    void convert_from_zmq(zmq::multipart_t &msg) 
    {
        timestamp_ = msg.poptyp<util::HighResTimePoint>();
        auto leftover_msg = msg.pop();
        data_ = *static_cast<DATA_T*>(leftover_msg.data());
    }

    template < typename TT = DATA_T, std::enable_if_t < std::is_base_of_v<msg::ZMQConverter, TT>, int > = 0 >
    zmq::multipart_t convert_to_zmq() const 
    {
        zmq::multipart_t multi;
        multi.addtyp(timestamp_);
        multi.append(data_.to_zmq_msg());
        return multi;
    }

    template < typename TT = DATA_T, std::enable_if_t < !std::is_base_of_v<msg::ZMQConverter, TT>, int > = 0 >
    zmq::multipart_t convert_to_zmq() const 
    {
        zmq::multipart_t multi;
        multi.addtyp(timestamp_);
        multi.addtyp(data_);
        return multi;
    }

    DATA_T data_;
    util::HighResTimePoint timestamp_;
};

}