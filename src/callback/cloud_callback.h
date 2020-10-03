#pragma once

/**
 * 
 * @author Pascal Buschermöhle
 * 
 */

#include <util/process_thread.h> 
#include <registration/registration.h>
#include <util/concurrent_ring_buffer.h>
#include <msg/msgs_stamped.h>

namespace fastsense::callback
{

    using Registration = fastsense::registration::Registration;
    using PointCloudBuffer = fastsense::util::ConcurrentRingBuffer<fastsense::msg::PointCloudStamped>;

    class CloudCallback : public fastsense::util::ProcessThread{
        public:
            CloudCallback(Registration& registration, PointCloudBuffer& cloud_buffer);

            void start() override;

            void callback();

            void stop() override;

        private:
            Registration& registration;
            PointCloudBuffer& cloud_buffer;

    };
}