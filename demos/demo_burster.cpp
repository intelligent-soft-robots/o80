#include <unistd.h>
#include <iostream>
#include "o80/burster.hpp"

void run()
{
    std::string segment_id{"o80_demo_burster"};
    o80::Burster b(segment_id);

    while (true)
    {
        b.pulse();
        std::cout << "pulse !" << std::endl;
    }
}

int main()
{
    run();
}
