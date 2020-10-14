#pragma once

/**
 * @file imu_callback.h
 * @author Pascal Buschermöhle
 */

#include <util/process_thread.h> 
#include <registration/registration.h>
#include <util/concurrent_ring_buffer.h>
#include <msg/msgs_stamped.h>

namespace fastsense::callback
{

    using Registration = fastsense::registration::Registration;
    using ImuBuffer = fastsense::util::ConcurrentRingBuffer<fastsense::msg::ImuMsgStamped>;

    class ImuCallback : public fastsense::util::ProcessThread{
        public:
            ImuCallback(Registration& registration, std::shared_ptr<ImuBuffer>& imu_buffer);

            void start() override;

            void callback();

            void stop() override;

        private:
            Registration& registration;
            std::shared_ptr<ImuBuffer>& imu_buffer;

    };
}