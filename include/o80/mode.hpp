#pragma once

namespace o80
{
/**
 * @brief Possible mode for a command.
 * queue : will run after all previous commands
 * (corresponding to the same actuator) finished
 * overwrite : will stop and cancel all previous commands and run immediately
 */
enum Mode
{
    QUEUE,
    OVERWRITE
};
}
