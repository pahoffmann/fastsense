#include <iostream>
#include "app.h"

int main()
{
    App app;

    for(;;)
    {
        std::cout << app.getCurrentState()->description() << "\n";
        if(std::cin.get() == '\n')
        {
            app.next();
        }
    }

    return 0;
}
