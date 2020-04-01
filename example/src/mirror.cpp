#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <memory>
#include "o80/example/example.hpp"
#include "o80/front_end.hpp"

typedef o80::FrontEnd<o80_EXAMPLE_QUEUE_SIZE, 2, o80_example::Joint> Frontend;

std::atomic<bool> RUNNING(true);
void stop(int)
{
    RUNNING = false;
    std::cout << "dummy mirror: ctr+c signal detected\n";
}

int run(int from, int to)
{
    std::string segment_id_from = o80_example::get_segment_id(from);
    Frontend frontend_from(segment_id_from);

    std::string segment_id_to = o80_example::get_segment_id(to);
    Frontend frontend_to(segment_id_to);

    int previous_iteration = -1;
    int from_iteration = -1;

    while (RUNNING)
    {
        // waiting for origin robot to iterate at least once
        while (RUNNING && from_iteration == previous_iteration)
        {
            from_iteration = frontend_from.read().get_iteration();
            usleep(1);
        }
        previous_iteration = from_iteration;

        // creating command for setting mirrored robot
        // to same state as origin robot
        o80::Observation<2, o80_example::Joint> observation =
            frontend_from.read();
        o80::States<2, o80_example::Joint> joints =
            observation.get_observed_states();

        for (int dof = 0; dof < 2; dof++)
        {
            int value = joints.get(dof).get();
            frontend_to.add_command(dof, value, o80::Mode::OVERWRITE);
        }

        // sending commands to mirrored robot
        frontend_to.pulse();
    }
}

int main(int argc, char *argv[])
{
    int id_from = atoi(argv[1]);
    int id_to = atoi(argv[2]);

    // running the server until ctrl+c
    struct sigaction stopping;
    stopping.sa_handler = stop;
    sigemptyset(&stopping.sa_mask);
    stopping.sa_flags = 0;
    sigaction(SIGINT, &stopping, nullptr);
    RUNNING = true;
    int c = 0;

    run(id_from, id_to);
}
