#include <iostream>
#include <string>
#include "o80/memory_clearing.hpp"

void clear_memory(std::string segment_id)
{
    o80::clear_shared_memory(segment_id);
}

int main(int argc, char* argv[])
{
    if (argc < 2 || argc > 2)
    {
        std::cout << "usage: o80_clear_memory segment_id\n";
        return 1;
    }
    std::string segment_id{argv[1]};
    std::cout << "deleting shared memory related to segment id: " << segment_id
              << "\n";
    clear_memory(segment_id);
}
