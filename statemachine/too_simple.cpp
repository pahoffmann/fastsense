#include <csignal>
#include <iostream>

void signal_handler(int signal)
{
    std::cout << "ende\n";
    std::exit(1);
}

void wait_for_button()
{
    char dummy;
    while (true)
    {
        if (std::cin.get() == '\n') return;
    }
}

int main()
{
    std::signal(SIGINT, signal_handler);

    std::cout << "idle\n";
    wait_for_button();
    
    std::cout << "imu calib\n";

    while (true)
    {   
        wait_for_button();
        std::cout << "Slam started\n";
        // reset & start callback
        // reset & start imu accumulator

        wait_for_button();
        std::cout << "Stopped slam\n";
        // stop callback
        // stop imu accumulator
    }
}
