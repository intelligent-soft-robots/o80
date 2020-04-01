#include "o80/internal/command_type.hpp"

namespace o80
{
CommandType::CommandType(Speed _speed) : type(Type::SPEED), speed(_speed.value)
{
}

CommandType::CommandType(Iteration _iteration)
    : type(Type::ITERATION), iteration(_iteration.value)
{
}

CommandType::CommandType(Duration_us _duration_us)
    : type(Type::DURATION), duration(_duration_us.value)
{
}

CommandType::CommandType() : type(Type::DIRECT)
{
}

/*
bool CommandType::direct_passed() const
{
  return TimeStamp(delayed.stamp).passed();
  }*/
}
