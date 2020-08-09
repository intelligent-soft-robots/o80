

template <typename T, class Sub>
State<T, Sub>::State(T v) : value(v)
{
}

template <typename T, class Sub>
State<T, Sub>::State()
{
}

template <typename T, class Sub>
T State<T, Sub>::get() const
{
    return value;
}

template <typename T, class Sub>
void State<T, Sub>::set(T v)
{
    value = v;
}

template <typename T, class Sub>
std::string State<T, Sub>::to_string() const
{
    return std::string("");
}

template <typename T, class Sub>
bool State<T, Sub>::finished(const o80::TimePoint &start,
                             const o80::TimePoint &now,
                             const Sub &start_state,
                             const Sub &current_state,
                             const Sub &previous_desired_state,
                             const Sub &target_state,
                             const o80::Speed &speed) const
{
    return o80::finished(start,
                         now,
                         start_state.value,
                         previous_desired_state.value,
                         target_state.value,
                         speed);
}

template <typename T, class Sub>
Sub State<T, Sub>::intermediate_state(const o80::TimePoint &start,
                                      const o80::TimePoint &now,
                                      const Sub &start_state,
                                      const Sub &current_state,
                                      const Sub &previous_desired_state,
                                      const Sub &target_state,
                                      const o80::Speed &speed) const
{
    return o80::intermediate_state(start,
                                   now,
                                   start_state.value,
                                   previous_desired_state.value,
                                   target_state.value,
                                   speed);
}

template <typename T, class Sub>
Sub State<T, Sub>::intermediate_state(const o80::TimePoint &start,
                                      const o80::TimePoint &now,
                                      const Sub &start_state,
                                      const Sub &current_state,
                                      const Sub &previous_desired_state,
                                      const Sub &target_state,
                                      const o80::Duration_us &duration) const
{
    return o80::intermediate_state(start,
                                   now,
                                   start_state.value,
                                   previous_desired_state.value,
                                   target_state.value,
                                   duration);
}

template <typename T, class Sub>
Sub State<T, Sub>::intermediate_state(long int start_iteration,
                                      long int current_iteration,
                                      const Sub &start_state,
                                      const Sub &current_state,
                                      const Sub &previous_desired_state,
                                      const Sub &target_state,
                                      const o80::Iteration &iteration) const
{
    return o80::intermediate_state(start_iteration,
                                   current_iteration,
                                   start_state.value,
                                   previous_desired_state.value,
                                   target_state.value,
                                   iteration);
}
