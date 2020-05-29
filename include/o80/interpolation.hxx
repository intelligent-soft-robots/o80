

template <typename T>
bool finished(const o80::TimePoint &start,
              const o80::TimePoint &now,
              const T &start_state,
              const T &current_state,
              const T &target_state,
              const o80::Speed &speed)
{
  double value_diff = fabs(static_cast<double>(target_state) -
			   static_cast<double>(start_state));
  double duration_us = value_diff / speed.value;
  long int expected_end
      = std::chrono::duration_cast<Microseconds>(start).count() + duration_us;
    if (now.count() > expected_end)
    {
        return true;
    }
    return false;
}

template <typename T>
T intermediate_state(const o80::TimePoint &start,
                     const o80::TimePoint &now,
                     const T &start_state,
                     const T &current,
                     const T &target_state,
                     const o80::Speed &speed)
{
    double time_diff =
      static_cast<double>(time_diff_us(start,now)) / 1E6;  // seconds
    double value_diff = static_cast<double>(target_state - start_state);
    double desired;
    if (value_diff > 0)
    {
        desired = static_cast<double>(start_state) +
                  static_cast<double>(speed.value) * time_diff;
        if (desired > target_state)
        {
            return target_state;
        }
        return static_cast<T>(desired);
    }
    desired = static_cast<double>(start_state) -
              static_cast<double>(speed.value) * time_diff;
    if (desired < target_state)
    {
        return target_state;
    }
    return static_cast<T>(desired);
}

template <typename T>
T intermediate_state(const o80::TimePoint &start,
                     const o80::TimePoint &now,
                     const T &start_state,
                     const T &current,
                     const T &target_state,
                     const o80::Duration_us &duration)
{
    long int passed = o80::time_diff_us(start, now);
    if (passed > duration.value)
    {
        return target_state;
    }
    double ratio =
        static_cast<double>(passed) / static_cast<double>(duration.value);
    double total_diff = static_cast<double>(target_state - start_state);
    double value = total_diff * ratio;
    return start_state + static_cast<T>(value);
}

template <typename T>
T intermediate_state(long int iteration_start,
                     long int iteration_now,
                     const T &start_state,
                     const T &current_state,
                     const T &target_state,
                     const o80::Iteration &iteration)
{
    if (iteration_now >= iteration.value)
    {
        return target_state;
    }
    T total_state = target_state - start_state;
    int total_iteration = iteration.value - iteration_start;
    int diff_iteration = iteration_now - iteration_start;
    double ratio = static_cast<double>(diff_iteration) /
                   static_cast<double>(total_iteration);
    double diff_state = ratio * static_cast<double>(total_state);
    double desired_state = diff_state + static_cast<double>(start_state);
    T ds = static_cast<T>(desired_state);
    return ds;
}
