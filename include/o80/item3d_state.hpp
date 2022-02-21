#pragma once

#include "o80/state6d.hpp"

namespace o80
{
class Item3dState
{
    /*!
      State6d, but with the convention that
      the first 3 values are a 3d position, and
      the last 3 values a 3d velocity.
     */
public:
    Item3dState();
    Item3dState(
        double d1, double d2, double d3, double d4, double d5, double d6);
    Item3dState(const State6d &state6d);
    Item3dState(const std::array<double, 3> &p, const std::array<double, 3> &v);
    void set_position(double a, double b, double c);
    void set_velocity(double a, double b, double c);
    void set_position(const std::array<double, 3> &position);
    void set_velocity(const std::array<double, 3> &velocity);
    void set(double d1, double d2, double d3, double d4, double d5, double d6);
    std::array<double, 3> get_position() const;
    std::array<double, 3> get_velocity() const;
    double get(int index) const;
    std::string to_string() const;

    bool finished(const o80::TimePoint &start,
                  const o80::TimePoint &now,
                  const Item3dState &start_state,
                  const Item3dState &current_state,
                  const Item3dState &previous_desired_state,
                  const Item3dState &target_state,
                  const o80::Speed &speed) const
    {
        return state6d_.finished(start,
                                 now,
                                 start_state.state6d_,
                                 current_state.state6d_,
                                 previous_desired_state.state6d_,
                                 target_state.state6d_,
                                 speed);
    }

    Item3dState intermediate_state(const o80::TimePoint &start,
                                   const o80::TimePoint &now,
                                   const Item3dState &start_state,
                                   const Item3dState &current_state,
                                   const Item3dState &previous_desired_state,
                                   const Item3dState &target_state,
                                   const o80::Speed &speed) const
    {
        State6d r = state6d_.intermediate_state(start,
                                                now,
                                                start_state.state6d_,
                                                current_state.state6d_,
                                                previous_desired_state.state6d_,
                                                target_state.state6d_,
                                                speed);
        return Item3dState(r);
    }

    Item3dState intermediate_state(const o80::TimePoint &start,
                                   const o80::TimePoint &now,
                                   const Item3dState &start_state,
                                   const Item3dState &current_state,
                                   const Item3dState &previous_desired_state,
                                   const Item3dState &target_state,
                                   const o80::Duration_us &duration) const
    {
        State6d r = state6d_.intermediate_state(start,
                                                now,
                                                start_state.state6d_,
                                                current_state.state6d_,
                                                previous_desired_state.state6d_,
                                                target_state.state6d_,
                                                duration);
        return Item3dState(r);
    }

    Item3dState intermediate_state(long int start_iteration,
                                   long int current_iteration,
                                   const Item3dState &start_state,
                                   const Item3dState &current_state,
                                   const Item3dState &previous_desired_state,
                                   const Item3dState &target_state,
                                   const o80::Iteration &iteration) const
    {
        State6d r = state6d_.intermediate_state(start_iteration,
                                                current_iteration,
                                                start_state.state6d_,
                                                current_state.state6d_,
                                                previous_desired_state.state6d_,
                                                target_state.state6d_,
                                                iteration);
        return Item3dState(r);
    }

  double to_duration(double speed,
		     const Item3dState& target_state) const
{
  return state6d_.to_duration(speed,target_state.state6d_);
}
  
  
    template <class Archive>
    void serialize(Archive &archive)
    {
        archive(state6d_);
    }

public:
    State6d state6d_;
};

}  // namespace o80
