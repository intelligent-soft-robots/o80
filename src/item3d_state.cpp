#include "o80/item3d_state.hpp"

namespace o80
{
Item3dState::Item3dState()
{
}

Item3dState::Item3dState(
    double d1, double d2, double d3, double d4, double d5, double d6)
    : state6d_(d1, d2, d3, d4, d5, d6)
{
}

Item3dState::Item3dState(const std::array<double, 3>& p,
                         const std::array<double, 3>& v)
    : state6d_(p[0], p[1], p[2], v[0], v[1], v[2])
{
}

Item3dState::Item3dState(const State6d& state6d) : state6d_(state6d)
{
}

std::string Item3dState::to_string() const
{
    std::stringstream ss;
    ss << "position: " << state6d_.get<0>() << ", " << state6d_.get<1>() << ", "
       << state6d_.get<2>() << " | "
       << "velocity: " << state6d_.get<3>() << ", " << state6d_.get<4>() << ", "
       << state6d_.get<5>();
    return ss.str();
}

void Item3dState::set(
    double d1, double d2, double d3, double d4, double d5, double d6)
{
    state6d_.set<0>(d1);
    state6d_.set<1>(d2);
    state6d_.set<2>(d3);
    state6d_.set<3>(d4);
    state6d_.set<4>(d5);
    state6d_.set<5>(d6);
}

void Item3dState::set_position(double a, double b, double c)
{
    state6d_.set<0>(a);
    state6d_.set<1>(b);
    state6d_.set<2>(c);
}

void Item3dState::set_position(const std::array<double, 3>& position)
{
    state6d_.set<0>(position[0]);
    state6d_.set<1>(position[1]);
    state6d_.set<2>(position[2]);
}

void Item3dState::set_velocity(double a, double b, double c)
{
    state6d_.set<3>(a);
    state6d_.set<4>(b);
    state6d_.set<5>(c);
}

void Item3dState::set_velocity(const std::array<double, 3>& velocity)
{
    state6d_.set<3>(velocity[0]);
    state6d_.set<4>(velocity[1]);
    state6d_.set<5>(velocity[2]);
}

std::array<double, 3> Item3dState::get_position() const
{
    std::array<double, 3> r;
    r[0] = state6d_.get<0>();
    r[1] = state6d_.get<1>();
    r[2] = state6d_.get<2>();
    return r;
}

std::array<double, 3> Item3dState::get_velocity() const
{
    std::array<double, 3> r;
    r[0] = state6d_.get<3>();
    r[1] = state6d_.get<4>();
    r[2] = state6d_.get<5>();
    return r;
}

double Item3dState::get(int index) const
{
    if (index == 0) return state6d_.get<0>();
    if (index == 1) return state6d_.get<1>();
    if (index == 2) return state6d_.get<2>();
    if (index == 3) return state6d_.get<3>();
    if (index == 4) return state6d_.get<4>();
    return state6d_.get<5>();
}

}  // namespace o80
