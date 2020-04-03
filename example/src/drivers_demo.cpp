#include "o80/example/example.hpp"

int main()
{
    typedef o80_example::Action Action;
    typedef o80_example::Observation Observation;
    typedef o80_example::Driver Driver;

    typedef robot_interfaces::RobotBackend<Action, Observation> Backend;
    typedef robot_interfaces::SingleProcessRobotData<Action,
                                        Observation> Data;
    typedef robot_interfaces::RobotFrontend<Action, Observation> Frontend;

    // max time allowed for the robot to apply an action.
    double max_action_duration_s = 0.02;

    // max time allowed for 2 successive actions
    double max_inter_action_duration_s = 0.05;

    // demo showing the separated usage of backend and frontend
    {
        std::cout << "\n -- * -- Frontend and Backend -- * --\n" << std::endl;

        std::shared_ptr<Driver> driver_ptr = std::make_shared<Driver>(0, 1000);
        std::shared_ptr<Data> data_ptr = std::make_shared<Data>();

        Backend backend(driver_ptr,
                        data_ptr,
                        max_action_duration_s,
                        max_inter_action_duration_s);
        backend.initialize();

        Frontend frontend(data_ptr);

        Action action;
        Observation observation;

        // simulated action :
        // 1 dof going from 200 to 300
        // The other going from 300 to 200

        for (uint value = 200; value <= 300; value++)
        {
            action.values[0] = value;
            action.values[1] = 500 - value;
            // this action will be stored at index
            robot_interfaces::TimeIndex index =
                frontend.append_desired_action(action);
            // getting the observation corresponding to the applied
            // action, i.e. at the same index
            observation = frontend.get_observation(index);
            std::cout << "value: " << value << " | ";
            action.print(false);
            observation.print(true);
        }
    }
}
