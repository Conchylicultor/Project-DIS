#ifndef SUPERVISOR_CPP
#define SUPERVISOR_CPP

#include <array>

#include "../common/Config.hpp"
#include "../common/PSOParams.hpp"


/*
 * Configuration of an e-puck, such as position, rotation
 *
 * see https://www.cyberbotics.com/guide/using-numerical-optimization-methods.php
 */
struct RobotConfig
{
    std::array<double, 3> translation;
    std::array<double, 4> rotation;
};

// TODO change FLOCK_SIZE when merging "super" supervisors together
using RobotConfigs = std::array<RobotConfig, FLOCK_SIZE>;


#endif


// vim: set spelllang=en_gb
// vim: set ts=4 sw=4
