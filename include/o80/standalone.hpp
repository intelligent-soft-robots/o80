// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include "back_end.hpp"
#include "o80/frequency_manager.hpp"
#include "o80/burster.hpp"
#include "o80/internal/standalone_runner.hpp"
#include "o80/observation.hpp"
#include "o80/typedefs.hpp"
#include "observation.hpp"
#include "robot_interfaces/robot_backend.hpp"
#include "robot_interfaces/robot_data.hpp"
#include "robot_interfaces/robot_driver.hpp"
#include "robot_interfaces/robot_frontend.hpp"
#include "synchronizer/leader.hpp"
#include "typedefs.hpp"

namespace o80
{
void please_stop(std::string segment_id)
{
    shared_memory::set<bool>(segment_id, "should_stop", true);
    try
    {
        synchronizer::Leader leader(segment_id + "_synchronizer");
        leader.pulse();
    }
    catch (...)
    {
    }
}

/**
 * Standalone encapsulates 1) a o80 Backend, and
 * 2) robot_interfaces (RI) backend and frontend.
 * Desired states as computed at each iteration of the
 * o80 backend used to generate the actions that are set to the RI
 * frontend. The o80 backend also reads the observations from the
 * RI frontend and generates from them the observation
 * it writes to the shared memory.
 * Virtual convert functions should be implemented to set up the conversion
 * from o80 desired states to RI actions (it may be that the same classes are
 * used
 * as template for o80 desired state and RI action,in which case convert
 * function may do nothing)
 * Instances of o80 backend and robot_interfaces
 * front and backend are created and managed by Standalone.
 * When started, threads for the RI backend is started,
 * but o80 backend will only iterate upon calls to the spin functions
 * @tparam QUEUE_SIZE number of commands that can be hosted
 * in the command queue at any point of time. Exceptions will be
 * thrown if more commands are queued.
 * @tparam NB_ACTUATORS number of actuators of the robot
 * @tparam RI_ACTION action supported by the RI front and backends
 * @tparam RI_OBSERVATION observation read from RI frontend
 * @tparam o80_STATE class encapsulating the state of an
 * actuator of the robot
 * @tparam EXTENDED_STATE (optional) class encapsulating
 * supplementary arbitrary information that will be added
 * to o80 observations written in the shared memory
 */
template <int QUEUE_SIZE,
          int NB_ACTUATORS,
          class RI_ACTION,
          class RI_OBSERVATION,
          class o80_STATE,
          class o80_EXTENDED_STATE>
class Standalone
{
public:
    static constexpr int queue_size = QUEUE_SIZE;
    static constexpr int nb_actuators = NB_ACTUATORS;

    typedef RI_ACTION RiAction;
    typedef RI_OBSERVATION RiObservation;
    typedef o80_STATE o80State;
    typedef o80_EXTENDED_STATE o80ExtendedState;

private:
    typedef robot_interfaces::RobotData<RI_ACTION,
					RI_OBSERVATION,
					robot_interfaces::Status>
        RiData;
    typedef std::shared_ptr<RiData> RiDataPtr;
    typedef robot_interfaces::RobotBackend<RI_ACTION, RI_OBSERVATION> RiBackend;
    typedef std::shared_ptr<RiBackend> RiBackendPtr;
    typedef robot_interfaces::RobotFrontend<RI_ACTION, RI_OBSERVATION>
        RiFrontend;
    typedef robot_interfaces::RobotDriver<RI_ACTION, RI_OBSERVATION> RiDriver;
    typedef std::shared_ptr<RiDriver> RiDriverPtr;
    typedef o80::
        BackEnd<QUEUE_SIZE, NB_ACTUATORS, o80_STATE, o80_EXTENDED_STATE>
            o80Backend;

public:
    /**
     * Creates instances of o80 backend, and RI front/back ends based
     * on the templated parameter and the robot driver.
     * @param ri_driver robot_interfaces robot driver
     * documentation
     * @param frequency  in non burst mode, spin function will wait at each
     * iteration the
     * amount of time suitable for enforcing this frequency
     * @param segment_id shared memory segment id.
     * o80 frontend instance should use the same.
     */
    Standalone(RiDriverPtr ri_driver_ptr,
               double frequency,
               std::string segment_id);

    ~Standalone();

    /**
     * Starts the RI backend
     */
    void start();

    /**
     * Stops the RI backend
     */
    void stop();

    /**
     * If bursting is false, performs one iteration and then wait for the time
     * requied
     * to match the standalone frequency.
     * If bursting is true, performs n iterations, and then hang until the o80
     * frontend calls burst. "n" is the number of iterations passed to the o80
     * frontend burst function.
     *
     */
    bool spin(bool bursting = false);

    /**
     * similar to spin, and the extended state is added to the observation
     * that is written to the shared memory by the o80 backend
     *
     */
    bool spin(o80_EXTENDED_STATE& extended_state, bool bursting = false);

    /**
     * convert the observation read from RI frontend into current robot states
     * to be added to the o80 observation that will be written in the shared
     * memory
     */
    virtual o80::States<NB_ACTUATORS, o80_STATE> convert(
        const RI_OBSERVATION& observation) = 0;

    /**
     * convert the observation read from RI frontend into current robot states
     * to be added to the o80 observation that will be written in the shared
     * memory
     */
    virtual RI_ACTION convert(
        const o80::States<NB_ACTUATORS, o80_STATE>& states) = 0;

    /**
     * if implemented,
     * add information to extended state based on the observation read from
     * RI frontend. The extended state will be added to the o80 observation
     * written to the shared memory.
     */
    virtual void enrich_extended_state(o80_EXTENDED_STATE& extended_state,
				       const RI_OBSERVATION& observation) = 0;

private:
    bool iterate(const TimePoint& time_now, o80_EXTENDED_STATE& extended_state);

private:
    double frequency_;
    Microseconds period_;
    FrequencyManager frequency_manager_;
    TimePoint now_;
    std::shared_ptr<Burster> burster_;
    std::string segment_id_;
    RiDriverPtr ri_driver_ptr_;
    RiDataPtr ri_data_ptr_;
    RiFrontend ri_frontend_;
    RiBackend* ri_backend_ptr_;
    o80Backend o8o_backend_;
};

/**
 * @brief instantiates instances of RobotDriver and Standalone,
 * and starts them in a thread. A runtime exception is thrown
 * if another standalone of the same segment_id has already
 * been started.
 */
template <class RobotDriver,
	  class o80Standalone,
	  typename... Args>
void start_action_timed_standalone(std::string segment_id,
                                   double frequency,
                                   bool bursting,
                                   Args&&... args);

template <class RobotDriver,
	  class o80Standalone,
	  typename... Args>
void start_standalone(std::string segment_id,
                      double frequency,
                      bool bursting,
                      Args&&... args);

/**
 * @brief stop the standalone of the specified segment_id.
 * A runtime error is thrown if no such standalone is running.
 */
void stop_standalone(std::string segment_id);

/**
 * @brief returns true if the standalone is iterating.
 * (returns also false if the standalone does not exist)
 */
bool standalone_is_running(std::string segment_id);

#include "standalone.hxx"
}
