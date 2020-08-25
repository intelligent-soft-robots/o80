#pragma once

#include "o80/state.hpp"

namespace o80
{

  class State1d : public State<double,State1d>
  {
  public:
    State1d(): State<double,State1d>(){}
    State1d(double value): State<double,State1d>(value){}
  };

  
}
