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

using RobotConfigs = std::array<RobotConfig, FLOCK_SIZE * NUMBER_OF_FLOCKS>;


/*
 * Initialize flock position and devices.
 */
void reset();

/*
 * Read the configuration for all robots
 *
 * The configuration of the i-th robot is stored at the i-th index of the returned array
 */
RobotConfigs readAllRobotsConfig();


/*
 * Compute the fitness for the current PSO parameters
 */
double simulate(RobotConfigs const& initialConfigs, PSOParams const& params);


#endif


// vim: set spelllang=en_gb
// vim: set ts=4 sw=4
