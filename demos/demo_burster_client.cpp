#include <unistd.h>
#include "o80/burster.hpp"


// start demo_burster first

void run()
{
  std::string segment_id{"o80_demo_burster"};
  o80::BursterClient bc(segment_id);

  for(int iteration=0;iteration<3;iteration++)
    {
      std::cout << "burst 10 !" << std::endl;
      bc.burst(10);
      usleep(2e6);
    }

  bc.final_burst();
  
}


int main()
{
  run();
}
