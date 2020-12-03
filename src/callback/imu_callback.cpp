/**
 *
 * @author Pascal Buschermöhle
 *
 */


/*#include "imu_callback.h"

using namespace fastsense::callback;

ImuCallback::ImuCallback(Registration& registration, std::shared_ptr<ImuBuffer>& imu_buffer)
    : ProcessThread(),
      registration{registration},
      imu_buffer{imu_buffer} {}

void ImuCallback::thread_run()
{
    fastsense::msg::ImuStamped imu;
    while (running)
    {
        if (imu_buffer->pop_nb(&imu, DEFAULT_POP_TIMEOUT))
        {
            registration.update_imu_data(imu);
        }
    }
}*/