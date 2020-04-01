#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include "o80/example/example.hpp"

using namespace o80_example;

std::atomic<bool> RUNNING(true);
void stop(int)
{
    RUNNING = false;
    std::cout << "o80 standalone example: ctr+c signal detected\n";
}

void run(int id, int frequency, bool bursting)
{
    std::string segment_id = o80_example::get_segment_id(id);
    o80::clear_shared_memory(segment_id);

    Driver driver;
    Standalone standalone(driver, static_cast<double>(frequency), segment_id);

    standalone.start();

    bool running = true;

    while (running && RUNNING)
    {
        running = standalone.spin(bursting);
    }

    standalone.stop();
}

int main(int argc, char *argv[])
{
    int id;
    int frequency;
    bool bursting = false;

    if (argc > 3)
    {
        id = atoi(argv[1]);
        frequency = atoi(argv[2]);
        int bursting_ = atoi(argv[3]);
        if (bursting_ > 0)
        {
            bursting = true;
        }
    }
    else
    {
        std::cout << "\nusage: example_standalone id frequency(int) bursting\n";
        return 1;
    }

    // running the server until ctrl+c
    struct sigaction stopping;
    stopping.sa_handler = stop;
    sigemptyset(&stopping.sa_mask);
    stopping.sa_flags = 0;
    sigaction(SIGINT, &stopping, nullptr);
    RUNNING = true;
    int c = 0;

    run(id, frequency, bursting);
}
