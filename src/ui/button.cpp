/**
 * @file Button.cpp
 * @author Marcel Flottmann
 * @date 2021-02-11
 */

#include "button.h"
#include <thread>

using namespace fastsense::ui;
using namespace std::chrono_literals;

Button::Button(const gpiod::line& line, std::chrono::nanoseconds timeout) :
    line_(line), timeout_(timeout)
{
    line_.request({"fastsense", gpiod::line_request::DIRECTION_INPUT, 0});
}

bool Button::wait_for_press_or_condition(std::function<bool()> condition)
{
    // empty event queue
    // while (line_.event_wait(0ns))
    // {
    //     line_.event_read();
    // }

    // wait for press...
    while (!line_.get_value())
    {
        // ...or condition
        if (condition())
        {
            return false;
        }
        std::this_thread::sleep_for(timeout_);
    }
    // clear event
    //line_.event_read();
    return true;
}