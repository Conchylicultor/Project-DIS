#include <array>
#include <algorithm>
#include <iostream>
#include <string>
#include <complex>
#include <cmath>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#include "supervisor.hpp"
#include "../common/Config.hpp"
#include "../common/PSOParams.hpp"

using namespace std;

// Some remark
// Each supervisor is associate with the flock corresponding to
// its name (super1 > flock 1, super2 > flock 2,...)
// Each flock has to contain the same number of robots (FLOCK_SIZE) !!!
// The epucks are automatically associated with the correct supervisor:
// epuck[    0      ;  FLOCK_SIZE-1] > super1
// epuck[FLOCK_SIZE ; 2*FLOCK_SIZE-1] > super2
// ...

// Print information
#define coutSuper std::cout << "[" << flockId << "]: "
#define PRINT_INFO if(printIter > 0 && printIter % frequencyPrint == 0)
const int frequencyPrint = 10000;
int printIter = 0;


// Fitness weights
static const double WEIGHT_ORIENTATION = 1.0;
static const double WEIGHT_COHESION = 3.0;
static const double WEIGHT_VELOCITY = 1.0;


struct Flock
{
    WbNodeRef robs[FLOCK_SIZE];
    WbFieldRef robs_translation[FLOCK_SIZE];
    WbFieldRef robs_rotation[FLOCK_SIZE];

    double loc[FLOCK_SIZE][4]; // Contain the localisation of all robots
    double centerOfMass[3] = { 0.0, 0.0, 0.0 };
    double prevCenterOfMass[3] = { 0.0, 0.0, 0.0 }; // Location of the center of mass at time (t-1)
    double migrationUrge[3] = { 1.0, 0.0, 0.0 };    // Vector containing the direction of the urge
};

std::array<Flock, NUMBER_OF_FLOCKS> flocks;

WbDeviceTag emitter; // This emitter is only used to send e-puck their PSO settings

// -------------------
// Some math functions
// -------------------

/*
 * Compute the Euclidean distance between two points
 */
double computeDist(double x[3],double y[3])
{
  return std::sqrt((x[0]-y[0]) * (x[0]-y[0]) +
                   (x[1]-y[1]) * (x[1]-y[1]) +
                   (x[2]-y[2]) * (x[2]-y[2]));
}

/*
 * Compute the dot product
 */
double computeDot(double x[3],double y[3])
{
  return x[0]*y[0] + x[1]*y[1] + x[2]*y[2];
}

// -------------------
// Fitness functions
// -------------------

/*
 * Compute orientation metric.
 */
double getOrientation(std::size_t flockId)
{
  std::complex<double> orientation = 0.0;
  for(int i=0 ; i<FLOCK_SIZE ; ++i)
  {
    orientation += std::polar(1.0, flocks[flockId].loc[i][3]); // 1.0 * exp(i*theta)
  }

  // Normalize
  return std::abs(orientation) / FLOCK_SIZE;
}

/*
 * Compute cohesion metric.
 */
double getCohesion(std::size_t flockId)
{
  // The center of mass is already computed in the main function

  // Compute the average distance to the center of mass
  double dist = 0.0;
  for(int i=0 ; i<FLOCK_SIZE ; ++i)
  {
    dist += computeDist(flocks[flockId].centerOfMass, flocks[flockId].loc[i]); // Thanks to the dirty C syntax, loc[i][4] is automatically "cast" into loc[i][3]
  }
  dist /= FLOCK_SIZE;

  // Return the metric
  return 1.0/(1.0+dist); // The closest, the better is
}

/*
 * Compute velocity metric.
 */
double getVelocity(std::size_t flockId)
{
  const double maxVelocity = 0.000104 * TIME_STEP; // [m/TIME_STEP]

  // Compute the current velocity
  double velocityVector[3] = {flocks[flockId].centerOfMass[0] - flocks[flockId].prevCenterOfMass[0],
                              flocks[flockId].centerOfMass[1] - flocks[flockId].prevCenterOfMass[1],
                              flocks[flockId].centerOfMass[2] - flocks[flockId].prevCenterOfMass[2]};
  // = x(t) - x(t-1)

  // First way: Compute the projection with respect to the migration urge
  //double currentVelocity = computeDot(velocityVector, migrationUrge);

  // Second way: We simply take the velocity of the center of mass
  double origin[3] = {0,0,0};
  double currentVelocity = computeDist(velocityVector, origin); // Norm of the vector

  return currentVelocity / maxVelocity; // Return the normalized velocity
}

/*
 * Compute performance metric.
 */
double computeFitnessStep(std::size_t flockId)
{
  double orientation = getOrientation(flockId);
  double cohesion = getCohesion(flockId);
  double velocity = getVelocity(flockId);

  PRINT_INFO
  {
    coutSuper << "o:" << orientation << endl;
    coutSuper << "c:" << cohesion << endl;
    coutSuper << "v:" << velocity << endl;
  }

  // Return normalized and weighted fitness
  return std::pow(orientation, WEIGHT_ORIENTATION) *
         std::pow(cohesion, WEIGHT_COHESION) *
         std::pow(velocity, WEIGHT_VELOCITY);
}


// -------------------
// Reset functions
// -------------------


/*
 * Read the current configuration for this robot
 */
RobotConfig readRobotConfig(std::string const& robotName)
{
    RobotConfig config;

    // 'supervisor' here is NOT the PSO supervisor!
    WbNodeRef node = wb_supervisor_node_get_from_def(robotName.c_str());

    WbFieldRef transField = wb_supervisor_node_get_field(node, "translation");
    double const* translation = wb_supervisor_field_get_sf_vec3f(transField);
    std::copy_n(translation, 3, std::begin(config.translation));

    WbFieldRef rotField = wb_supervisor_node_get_field(node, "rotation");
    double const* rotation = wb_supervisor_field_get_sf_rotation(rotField);
    std::copy_n(rotation, 4, std::begin(config.rotation));

    return config;
}

/*
 * Reset this robot's position, orientation, ...
 */
void resetRobot(std::string const& robotName, RobotConfig const& config)
{
    // 'supervisor' here is NOT the PSO supervisor!
    WbNodeRef node = wb_supervisor_node_get_from_def(robotName.c_str());

    WbFieldRef transField = wb_supervisor_node_get_field(node, "translation");
    wb_supervisor_field_set_sf_vec3f(transField, config.translation.data());

    WbFieldRef rotField = wb_supervisor_node_get_field(node, "rotation");
    wb_supervisor_field_set_sf_rotation(rotField, config.rotation.data());

    // NOTE: motors are not reset. This might create a slight imperfection for the
    // few first iterations but nothing significant for the overall fitness measure.
}

/*
 * Get the name of the i-th robot
 */
std::string getRobotName(std::size_t i)
{
    return "epuck" + std::to_string(i);
}

/*
 * Read the configuration for all robots
 *
 * The configuration of the i-th robot is stored at the i-th index of the returned array
 */
RobotConfigs readAllRobotsConfig()
{
    RobotConfigs configs; // RVO

    for (std::size_t i = 0; i < configs.size(); ++i)
    {
        std::string name = getRobotName(i);
        configs[i] = readRobotConfig(name);
    }

    return configs;
}

/*
 * Reset all robots
 */
void resetAllRobots(RobotConfigs const& configs)
{
    for (std::size_t i = 0; i < configs.size(); ++i)
    {
        std::string name = getRobotName(i);
        resetRobot(name, configs[i]);
    }

    // Finally, reset the physics module
    wb_supervisor_simulation_reset_physics();
}


// -------------------
// PSO functions
// -------------------

/*
 * Send the given parameters to all robots
 */
void sendParamsToRobots(PSOParams const& params)
{
    PSOParams restoredParams = restoreParams(params); // De-normalize the params
    void const* buffer = reinterpret_cast<void const*>(&restoredParams);
    int const size = sizeof(PSOParams);

    wb_emitter_send(emitter, buffer, size);
}


// -------------------
// Main functions
// -------------------

/*
 * Initialize flock position and devices.
 */
void reset(void)
{
  wb_robot_init();

  emitter = wb_robot_get_device("emitter");
  wb_emitter_set_channel(emitter, SUPER_EPUCK_CHANNEL);

  for (int flockId = 0; flockId < NUMBER_OF_FLOCKS; ++flockId)
  {
      auto& flock = flocks[flockId];

      for (int i = 0; i < FLOCK_SIZE; i++)
      {
          string robotName = "epuck" + std::to_string(i + flockId * FLOCK_SIZE);

          flock.robs[i] = wb_supervisor_node_get_from_def(robotName.c_str());
          if (flock.robs[i] == NULL) // Check correctness
          {
              cout << "ERROR: Wrong number of epuck: " << robotName << endl;
              exit(-1);
          }
          flock.robs_translation[i] = wb_supervisor_node_get_field(flock.robs[i], "translation");
          flock.robs_rotation[i] = wb_supervisor_node_get_field(flock.robs[i], "rotation");
      }
  }

  cout << FLOCK_SIZE * NUMBER_OF_FLOCKS << " robots loaded!" << endl;

  // TODO: Compute the migration urge
  // The migration urge is normalized
}

/*
 * Compute the fitness for the current PSO parameters
 */
double simulate(RobotConfigs const& initialConfigs, PSOParams const& params)
{
  // Debug message
  // cout << "Simulates params: " << params << endl;
  
  // Re-initialize the robots with the current PSO settings
  resetAllRobots(initialConfigs);
  sendParamsToRobots(params); // The params are "De-normalized" inside the function

  double fitnessGlobal = 0.0;

  for (std::size_t nbTimestep = 0; nbTimestep < PSOParams::SIMULATION_STEPS; ++nbTimestep)
  {
      for (int flockId = 0; flockId < NUMBER_OF_FLOCKS; ++flockId)
      {
          auto& flock = flocks[flockId];

          // Update the robot locations
          for (int i = 0; i < FLOCK_SIZE; i++)
          {
              flock.loc[i][0] = wb_supervisor_field_get_sf_vec3f(flock.robs_translation[i])[0]; // x
              flock.loc[i][1] = wb_supervisor_field_get_sf_vec3f(flock.robs_translation[i])[1]; // y (probably useless)
              flock.loc[i][2] = wb_supervisor_field_get_sf_vec3f(flock.robs_translation[i])[2]; // z
              flock.loc[i][3] = wb_supervisor_field_get_sf_rotation(flock.robs_rotation[i])[3]; // theta (rotation)
          }

          // Update the center of mass
          for (int j = 0; j < 3; ++j) // 3 is the number of dimension
          {
              flock.centerOfMass[j] = 0.0f; // Reset
          }
          for (int i = 0; i < FLOCK_SIZE; ++i)
          {
              for (int j = 0; j < 3; ++j)
              {
                  // We add the contribution of each robot
                  flock.centerOfMass[j] += flock.loc[i][j] / FLOCK_SIZE;
              }
          }

          if (nbTimestep > 0) // no speed at time 0
          {
              double fitnessInstant = computeFitnessStep(flockId);

              // Update fitness
              if (nbTimestep > 0)
              {
                  // Plot every x timesteps
                  PRINT_INFO
                  {
                      coutSuper << "Performances of flock " << flockId << ":" << endl;
                      coutSuper << fitnessInstant << " (instant)" << endl;
                  }
              }

              fitnessGlobal += fitnessInstant;
              PRINT_INFO
              {
                  coutSuper << "Performances overall:" << endl;
                  coutSuper << fitnessGlobal / nbTimestep / NUMBER_OF_FLOCKS << endl;
              }
          }

          // (t-1) = t
          for (int j = 0; j < 3; ++j) // 3 is the number of dimension
          {
              flock.prevCenterOfMass[j] = flock.centerOfMass[j];
          }
      }


      ++printIter; // Update the printing iterator

      wb_robot_step(TIME_STEP);
  }

  return fitnessGlobal / PSOParams::SIMULATION_STEPS / NUMBER_OF_FLOCKS;
}


