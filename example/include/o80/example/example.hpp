#pragma once

#include "o80/standalone.hpp"
#include "robot_interfaces/robot_driver.hpp"

#define o80_EXAMPLE_SEGMENT "o80_EXAMPLE"
#define o80_EXAMPLE_QUEUE_SIZE 5000
#define o80_EXAMPLE_NB_DOFS 2

namespace o80_example
{
// Data flow

// sending commands
// ----------------
//
//  client ---> commands ---> o80 frontend
//     ---> (shared_memory) ---> o80 backend ---> (desired) o80 Joints --->
//     (convert)
//       ---> Action ---> robot driver ---> robot frontend ---> robot backend

// reading observations
// --------------------
//
//  robot backend ---> robot frontend ---> driver ---> observation
//     ---> (convert) ---> observation
//       ---> o80 backend ---> (shared memory) ---> o80 frontend ---> client

// to get this to work, the following classes are defined below:
// Driver, Action , Observation, Joints
// + Standalone (convert functions)

// Actions to be performed by robot, will be received by Driver
class Action
{
public:
    void print(bool endl)
    {
        std::cout << "action: " << values[0] << "\t" << values[1] << " ";
        if (endl) std::cout << std::endl;
    }

    std::string to_string() const
    {
        return std::to_string(values[0]) + " " + std::to_string(values[1]);
    }

    int values[2];
};

// Read from the robot by Driver
class Observation
{
public:
    void print(bool endl)
    {
        std::cout << "observation: " << values[0] << "\t" << values[1];
        if (endl) std::cout << std::endl;
    }

    int values[2];
};

// Communicate with the robot.
class Driver : public robot_interfaces::RobotDriver<Action, Observation>
{
public:
    Driver(int min, int max) : min_(min), max_(max)
    {
    }

    // at init dof are at min value
    void initialize()
    {
        values_[0] = min_;
        values_[1] = min_;
    }

    // just clip desired values
    // between 0 and 1000
    Action apply_action(const Action &desired_action)
    {
        Action applied;
        for (unsigned int i = 0; i < 2; i++)
        {
            if (desired_action.values[i] > max_)
            {
                applied.values[i] = max_;
            }
            else if (desired_action.values[i] < min_)
            {
                applied.values[i] = min_;
            }
            else
            {
                applied.values[i] = desired_action.values[i];
            }
            usleep(10);
            values_[i] = applied.values[i];
        }
        return applied;
    }

    Observation get_latest_observation()
    {
        Observation observation;
        observation.values[0] = values_[0];
        observation.values[1] = values_[1];
        return observation;
    }

    void shutdown()
    {
    }

    std::string get_error()
    {
        return "";
    }

private:
    int values_[2];
    int min_;
    int max_;
};

/*
 * o80 interface
 *
 * o80 allows to control each dof separately via commands.
 * The joint class provides functions that will be used by controllers
 * to transit from a (joint) start state to a (joint) target state
 * at a specified speed or such as reaching the target state
 * at a given iteration
 *
 */

// Desired state for a single dof
class Joint
{
public:
    Joint(int value_) : value(value_)
    {
    }

    Joint()
    {
    }

    static Joint eval(int value)
    {
        return Joint(value);
    }

    std::string to_string() const
    {
        return std::to_string(value);
    }

    int get() const
    {
        return value;
    }

    void set(int v)
    {
        value = v;
    }

    bool finished(const o80::TimePoint &start,
                  const o80::TimePoint &now,
                  const long int duration_us) const
    {
        o80::TimePoint finish(start.count() + duration_us);
        return finish > now;
    }

    bool finished(const o80::TimePoint &start,
                  const o80::TimePoint &now,
                  const Joint &start_state,
                  const Joint &current_state,
                  const Joint &previous_desired_state,
                  const Joint &target_state,
                  const o80::Speed &speed) const
    {
        long int start_value = start_state.value;
        long int end_value = target_state.value;
        long int value_diff = abs(end_value - start_value);
        double duration_sec = static_cast<double>(value_diff) /
                              static_cast<double>(fabs(speed.value));
        long int duration = static_cast<long int>(duration_sec * 1E6);
        long int expected_end_int = start.count() + duration;
        o80::TimePoint expected_end(expected_end_int);
        if (now > expected_end)
        {
            return true;
        }
        return false;
    }

    Joint intermediate_state(const o80::TimePoint &start,
                             const o80::TimePoint &now,
                             const Joint &start_state,
                             const Joint &current,
                             const Joint &previous_desired_state,
                             const Joint &target_state,
                             const o80::Speed &speed) const
    {
        double time_diff =
            static_cast<double>((now - start).count()) / 1E6;  // seconds
        long int start_value = start_state.value;
        long int end_value = target_state.value;
        long int value_diff = end_value - start_value;
        double desired;
        if (value_diff > 0)
        {
            desired =
                start_value + static_cast<double>(speed.value) * time_diff;
            if (desired > end_value)
            {
                return end_value;
            }
            return Joint(static_cast<int>(desired + 0.5));
        }
        desired = start_value - static_cast<double>(speed.value) * time_diff;
        if (desired < end_value)
        {
            return end_value;
        }
        return Joint(static_cast<int>(desired + 0.5));
    }

    Joint intermediate_state(const o80::TimePoint &start,
                             const o80::TimePoint &now,
                             const Joint &start_state,
                             const Joint &current,
                             const Joint &previously_desired_state,
                             const Joint &target_state,
                             const o80::Duration_us &duration) const
    {
        throw std::runtime_error("to implement !");
        return Joint();
    }

    Joint intermediate_state(long int iteration_start,
                             long int iteration_now,
                             const Joint &start_state,
                             const Joint &current_state,
                             const Joint &previous_desired_state,
                             const Joint &target_state,
                             const o80::Iteration &iteration) const
    {
        if (iteration_now >= iteration.value)
        {
            return Joint(target_state.value);
        }
        int total_state = target_state.value - start_state.value;
        int total_iteration = iteration.value - iteration_start;
        int diff_iteration = iteration_now - iteration_start;
        double ratio = static_cast<double>(diff_iteration) /
                       static_cast<double>(total_iteration);
        double diff_state = ratio * static_cast<double>(total_state);
        double desired_state =
            diff_state + static_cast<double>(start_state.value);
        int desired = static_cast<int>(desired_state + 0.5);
        return Joint(desired);
    }

    template <class Archive>
    void serialize(Archive &archive)
    {
        archive(value);
    }

    int value;

private:
    friend shared_memory::private_serialization;
};

/*
 * o80 interface and robot interface bridge
 *
 * o80 provides 2 desired joint states at each iteration.
 * (one per dof)
 * Robot interface requires Action, which encapsulate
 * the desired values for the 2 dofs.
 * Doing the conversion here
 *
 */

class Standalone
    : public o80::Standalone<o80_EXAMPLE_QUEUE_SIZE,  // Queue size
                             // for command passing
                             2,            // 2 dofs robot
                             Action,       // robot_interface action
                             Observation,  //  robot_interface observation
                             Joint,        // o80 observation
                             o80::EmptyExtendedState>  // no info on top of obs
{
public:
    Standalone(std::shared_ptr<Driver> driver_ptr,
               double max_action_duration_s,
               double max_inter_action_duration_s,
               double frequency,
               std::string segment_id)
        : o80::Standalone<o80_EXAMPLE_QUEUE_SIZE,
                          2,
                          Action,
                          Observation,
                          Joint,
                          o80::EmptyExtendedState>(driver_ptr,
                                                   max_action_duration_s,
                                                   max_inter_action_duration_s,
                                                   frequency,
                                                   segment_id)
    {
    }

    Standalone(std::shared_ptr<Driver> driver_ptr,
               double frequency,
               std::string segment_id)
        : o80::Standalone<o80_EXAMPLE_QUEUE_SIZE,
                          2,
                          Action,
                          Observation,
                          Joint,
                          o80::EmptyExtendedState>(
              driver_ptr,
              std::numeric_limits<double>::infinity(),
              std::numeric_limits<double>::infinity(),
              frequency,
              segment_id)
    {
    }

    Action convert(const o80::States<2, Joint> &joints)
    {
        Action action;
        for (size_t dof = 0; dof < 2; dof++)
        {
            const Joint &joint = joints.get(dof);
            action.values[dof] = joint.value;
        }
        return action;
    }

    // robot_interface observation and o80 observation
    // are the same thing
    o80::States<2, Joint> convert(const Observation &observation)
    {
        o80::States<2, Joint> states;
        states.set(0, Joint(observation.values[0]));
        states.set(1, Joint(observation.values[1]));
        return states;
    }
};

std::string get_segment_id(int id)
{
    std::string segment_id(o80_EXAMPLE_SEGMENT);
    segment_id = segment_id + std::string("_") + std::to_string(id);
    return segment_id;
}
}
