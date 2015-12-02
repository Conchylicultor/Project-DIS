#include <cassert>
#include <iostream>
#include <iterator>
#include <cstdio> // For sscanf
#include <cmath>
#include <string>

#include <webots/robot.h>

#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>

#include "../common/Config.hpp"
#include "../common/PSOParams.hpp"
#include "vec2.h"

using namespace std;


// Some remarks:
// Probably only works in a noise free environment
// Works if all the robots of the flock are within the communication area (but this
// point could be corrected easily if the robots only align with its neighbours)


string robotName;
int robotIdGlobal = 0; // World id
int flockId = 0; // Id of the flock
int robotId = 0; // Id in the flock

static const int NB_SENSORS = 8; // Number of distance sensors

WbDeviceTag ds[NB_SENSORS];     // Handle for the infrared distance sensors
WbDeviceTag receiver;           // Handle for the receiver node
WbDeviceTag emitter;            // Handle for the emitter node
WbDeviceTag superReceiver;      // Handle for the receiver node (supervisor -> epuck communication)

// -------------------
// Coordinate variables
// -------------------

Vec2 myPosition(0.0,0.0);
Vec2 myPrevPosition(0.0,0.0);
Vec2 mySpeed(0.0,0.0);
double myTheta = 0.0; // TODO we could use a compass instead

Vec2 neighboursPos[FLOCK_SIZE]; // Relative positions of the neighbours
Vec2 neighboursPrevPos[FLOCK_SIZE];
Vec2 neighboursRelativeSpeed[FLOCK_SIZE];

Vec2 migrationVec(0.0,-20.0); // TODO: Change our migration vector ?

// -------------------
// Obstacle avoidance functions
// -------------------

// Braitenberg parameters
const int MIN_SENS = 350; // Minimum sensibility value
const int MAX_SENS = 4096; // Maximum sensibility value
const int weightMatrix[2][NB_SENSORS] = {{-72,-58,-36,8,10,36,28,18},
                                         {17,29,34,10,8,-38,-56,-76}}; // Braitenberg weight

// const int weightMatrix[2][NB_SENSORS] = {{-72,-58,-36,8,10,36,28,18},
//                                         {18,28,36,10,8,-36,-58,-72}}; // Braitenberg weight

/*
 * Try avoiding obstacles
 */
void braitenbergObstacle(int wheelSpeed[2], bool &thresholdSpeedInstinct, int &maxSensorValue)
{
  // Reinitialisation
  int distance = 0;
  int sumSensors = 0;
  maxSensorValue = 0;

  wheelSpeed[0] = 0;
  wheelSpeed[1] = 0;

  // Compute the weighted behavior
  for(int i=0 ; i<NB_SENSORS ; i++)
  {
    distance = wb_distance_sensor_get_value(ds[i]); //Read sensor values

    // Weighted sum of distance sensor values for Braitenberg vehicle
    wheelSpeed[0] += weightMatrix[0][i] * distance; // Left
    wheelSpeed[1] += weightMatrix[1][i] * distance; // Right

    // Update the maximum sensor value and sumSensors
    if(distance > maxSensorValue)
    {
      maxSensorValue = distance;
    }
    sumSensors += distance;
  }

  // Adapt the speed
  wheelSpeed[0] /= MIN_SENS;
  wheelSpeed[1] /= MIN_SENS;

  // Define the speed instinct
  if (sumSensors > NB_SENSORS*MIN_SENS)
  {
    thresholdSpeedInstinct = true;
  }
  else
  {
    thresholdSpeedInstinct = false;
  }
}

// -------------------
// Wheels related functions
// -------------------

// Parameters of the wheels
const double AXLE_LENGTH = 0.052; // Distance between wheels of robot (meters)
const double SPEED_UNIT_RADS = 0.00628; // Conversion factor from speed unit to radian per second
const double WHEEL_RADIUS = 0.0205; // Wheel radius (meters)

const int MAX_WHEEL_SPEED = 800; // Maximum wheel speed

/*
 * Update myPosition using the wheel speed
 */
void updateCurrentPosition(const int wheelSpeed[2])
{
  double theta = myTheta;

  // Compute deltas of the robot
  double dl = wheelSpeed[0] * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
  double dr = wheelSpeed[1] * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
  double du = (dr + dl)/2.0;
  double dtheta = (dr - dl)/AXLE_LENGTH;

  // Compute deltas in the environment
  double dx = -du * std::sin(theta);
  double dz = -du * std::cos(theta);

  // Update position
  myPosition[0] += dx;
  myPosition[1] += dz;
  myTheta += dtheta;

  // Keep orientation within 0, 2pi
  if (myTheta > 2*M_PI)
  {
    myTheta -= 2.0*M_PI;
  }

  if (myTheta < 0)
  {
    myTheta += 2.0*M_PI;
  }
}

/*
 * Keep given int number within interval {-limit, limit}
 */
void checkLimit(int &number, int limit)
{
  if (number > limit)
    number = limit;
  if (number < -limit)
    number = -limit;
}

/*
 * Update the wheel speed from the desired robot speed and orientation
 */
void computeWheelSpeeds(int wheelSpeed[2], const Vec2 &robotSpeed, double robotOrientation)
{
  // Compute wanted position from Reynold's speed and current location
  double x =  robotSpeed[0]*std::cos(robotOrientation) - robotSpeed[1]*std::sin(robotOrientation); // x in robot coordinates
  double z = -robotSpeed[0]*std::sin(robotOrientation) - robotSpeed[1]*std::cos(robotOrientation); // z in robot coordinates

  double Ku = 0.2;   // Forward control coefficient
  double Kw = 10.0;  // Rotational control coefficient
  double range = std::sqrt(x*x + z*z);	// Distance to the wanted position
  double bearing = -std::atan2(x, z);	// Orientation of the wanted position

  // Compute forward control
  double u = Ku*range*std::cos(bearing);
  // Compute rotational control
  double w = Kw*range*std::sin(bearing);

  // Convert to wheel speeds!
  wheelSpeed[0] = 50*(u - AXLE_LENGTH*w/2.0) / WHEEL_RADIUS;
  wheelSpeed[1] = 50*(u + AXLE_LENGTH*w/2.0) / WHEEL_RADIUS;

  checkLimit(wheelSpeed[0], MAX_WHEEL_SPEED);
  checkLimit(wheelSpeed[1], MAX_WHEEL_SPEED);
}

// -------------------
// Communication functions
// -------------------

/*
 * Send a ping in order to let other robots know where this robot is
 */
void ping()
{
  string message = std::to_string(flockId) + "_" + std::to_string(robotId); // Send our id (and flock id)
  wb_emitter_send(emitter, message.c_str(), message.size()+1); // We add +1 for the '\0' caractere
}

/*
 * Receive messages from other and extract interesting informations
 */
void pong()
{
  // Read all packets available; because we have a null-delay this works fine
  // but might be an issue with real-world e-pucks
  for (; wb_receiver_get_queue_length(receiver) > 0; wb_receiver_next_packet(receiver))
  {
    // Read received data
    string receivedMessage = static_cast<const char*>(wb_receiver_get_data(receiver));
    const double* dir      = wb_receiver_get_emitter_direction(receiver);
    double signalStrength  = wb_receiver_get_signal_strength(receiver);

    // Compute and extract the robot id
    int receivedRobotId;
    int receivedFlockId;
    sscanf(receivedMessage.c_str(),"%d_%d", &receivedFlockId, &receivedRobotId);

    if(receivedFlockId != flockId) // We ignore robots from other flock
    {
      continue;
    }

    // Compute the position & cie
    double dirX = dir[0];
    double dirZ = dir[2];

    double theta = -std::atan2(dirZ, dirX);
    theta += myTheta; // Relative orientation of our neighbour

    double distance = std::sqrt(1.0 / signalStrength);

    neighboursPrevPos[receivedRobotId] = neighboursPos[receivedRobotId];

    neighboursPos[receivedRobotId] = { std::cos(theta) * distance,
                                       -std::sin(theta) * distance };

    neighboursRelativeSpeed[receivedRobotId] = (neighboursPos[receivedRobotId] - neighboursPrevPos[receivedRobotId])/DELTA_T;

    //cout << "-----------------------------------" << endl;
    //cout << "Robot " << receivedFlockId << " from flock " << receivedRobotId << " : " << endl;
    //cout << "Prev: " << neighboursPrevPos[receivedRobotId] << endl;
    //cout << "Current: " << neighboursPos[receivedRobotId] << endl;
    //cout << "Vel: " << neighboursRelativeSpeed[receivedRobotId] << endl;
  }
}

/*
 * Receive PSO settings from supervisor
 */
PSOParams getParamsFromSupervisor()
{
    PSOParams params; // RVO

    // Wait for data
    while (wb_receiver_get_queue_length(superReceiver) == 0)
    {
        wb_robot_step(TIME_STEP);
    }

    void const* buffer = wb_receiver_get_data(superReceiver);
    auto size = wb_receiver_get_data_size(superReceiver);

    // Make sure we receive something somewhat valid
    assert(buffer && size == sizeof(PSOParams));

    PSOParams const* p = reinterpret_cast<PSOParams const*>(buffer);
    params = *p; // copy what we got from the supervisor

    // Get ready for the next packet:
    wb_receiver_next_packet(superReceiver);

    return params;
}

// -------------------
// Reynolds functions
// -------------------


/*
 * Compute the desired vector speed using Reynolds rules
 */
void reynoldsRules(const PSOParams &params)
{
  Vec2 relAvgLoc;	// Flock average positions
  Vec2 relAvgSpeed;	// Flock average speeds

  Vec2 cohesion;
  Vec2 dispersion;
  Vec2 consistency;

  // Compute averages over the whole flock
  for(int i=0 ; i<FLOCK_SIZE ; i++)
  {
    // Don't consider yourself for the average
    if (i != robotId)
    {
      relAvgSpeed += neighboursRelativeSpeed[i];
      relAvgLoc += neighboursPos[i];
    }
  }
  relAvgSpeed /= (FLOCK_SIZE-1);
  relAvgLoc /= (FLOCK_SIZE-1); // -1 because we don't take ourself in account


  // Rule 1 - Aggregation/Cohesion: move towards the center of mass

  if (norm(relAvgLoc) > params.cohesionThreshold) // If center of mass is too far
  {
    cohesion = relAvgLoc ;  // Relative distance to the center of the swarm
  }

  // Rule 2 - Dispersion/Separation: keep far enough from flockmates

  for (int i=0 ; i<FLOCK_SIZE; i++)
  {
    if (i != robotId) // Loop on flockmates only
    {
      // If neighbor k is too close
      if (norm(neighboursPos[i]) < params.separationThreshold)
      {
        dispersion -= neighboursPos[i]; // Relative distance to k
      }
    }
  }

  // Rule 3 - Consistency/Alignment: match the speeds of flockmates
  consistency =  relAvgSpeed; // difference speed to the average

  // Aggregation of all behaviors with relative influence determined by weights
  mySpeed = cohesion * params.cohesionWeight;
  mySpeed +=  dispersion * params.spearationWeight;
  mySpeed +=  consistency * params.alignmentWeight;
  mySpeed += (migrationVec-myPosition) * params.migrationWeigth; // TODO: Change the migration policy ??
}

// -------------------
// Main functions
// -------------------


/*
 * Reset the robot's devices and get its ID
 */
static void reset()
{
    wb_robot_init();

    // Loading robot name
    robotName = wb_robot_get_name();
    sscanf(robotName.c_str(),"epuck%d",&robotIdGlobal); // read robot id from the robot's name
    robotId = robotIdGlobal % FLOCK_SIZE; // normalize between 0 and FLOCK_SIZE-1
    flockId = (robotIdGlobal/FLOCK_SIZE) + 1;

    cout << "Loading: robot " << robotIdGlobal << " (" << robotId << " in flock " << flockId << ")" << endl;

    // Loading R&B module
    receiver = wb_robot_get_device("receiver");
    superReceiver = wb_robot_get_device("superReceiver");
    emitter = wb_robot_get_device("emitter");
    wb_emitter_set_channel(emitter, EPUCK_EPCUK_CHANNEL);
    wb_receiver_set_channel(receiver, EPUCK_EPCUK_CHANNEL);
    wb_receiver_enable(receiver, TIME_STEP);
    wb_receiver_set_channel(superReceiver, SUPER_EPUCK_CHANNEL);
    wb_receiver_enable(superReceiver, TIME_STEP);

    // Loading the distances sensors
    for(int i=0; i<NB_SENSORS;i++) {
        ds[i]=wb_robot_get_device(string("ps" + std::to_string(i)).c_str());	// the device name is specified in the world file
        wb_distance_sensor_enable(ds[i], TIME_STEP);
    }
}

/*
 * Reinitialise local variables before a pso iteration
 */
void reset_run()
{
    // Loading the distances sensors
    for(int i=0; i<NB_SENSORS;i++) {
        ds[i]=wb_robot_get_device(string("ps" + std::to_string(i)).c_str());	// the device name is specified in the world file
        wb_distance_sensor_enable(ds[i], TIME_STEP);
    }
    
    // Reset all relative coordinates
    myPosition = Vec2(0.0,0.0);
    myPrevPosition = Vec2(0.0,0.0);
    mySpeed = Vec2(0.0,0.0);
    myTheta = 0.0;
    
    for(int i = 0 ; i < FLOCK_SIZE ; ++i)
    {
      neighboursPos[i] = Vec2(0.0,0.0);
      neighboursPrevPos[i] = Vec2(0.0,0.0);
      neighboursRelativeSpeed[i] = Vec2(0.0,0.0);
    }
}

/*
 * Apply simulation with the given parameters
 */
void simulate(PSOParams const& params)
{
    std::cout << "Starting simulation with PSO parameters: " << params << std::endl;

    int wheelSpeed[2] = { 0, 0 };            // Left and right wheel speed
    int wheelSpeedBraitenberg[2] = { 0, 0 }; // Left and right wheel speed

    bool thresholdSpeedInstinct = false;
    int maxSensorValue = 0;

    for (std::size_t i = 0; i < PSO_ITERATIONS; ++i) {
        // Braitenberg obstacle avoidance
        braitenbergObstacle(wheelSpeedBraitenberg, thresholdSpeedInstinct, maxSensorValue);

        // Emission/reception between flock members
        ping(); // Indicate our presence
        pong(); // Get informations from other robots

        // Update position
        myPrevPosition = myPosition;
        updateCurrentPosition(wheelSpeed);

        // Use received information for the Reynolds behavior
        reynoldsRules(params);

        // Compute the wheels speed
        // Use the desired speed and orientation to compute the wheelSpeed
        computeWheelSpeeds(wheelSpeed, mySpeed, myTheta);

        // Add Braitenberg (weighted with Reynolds)
        if (thresholdSpeedInstinct)
        {
            wheelSpeed[0] -= wheelSpeed[0] * maxSensorValue / (2 * MAX_SENS);
            wheelSpeed[1] -= wheelSpeed[1] * maxSensorValue / (2 * MAX_SENS);
        }
        wheelSpeed[0] += wheelSpeedBraitenberg[0];
        wheelSpeed[1] += wheelSpeedBraitenberg[1];

        // Set speed
        wb_differential_wheels_set_speed(wheelSpeed[0], wheelSpeed[1]);

        // Continue one step
        wb_robot_step(TIME_STEP);
    }
}

/*
 * Main function.
 */
int main()
{
    // Enable sensors and actuators, only once
    reset();

    // Main loop
    while (true)
    {
        // Wait for PSO parameters from supervisor
        PSOParams params = getParamsFromSupervisor();

        std::cout << "Received the following PSO parameters from supervisor: "
                  << params << std::endl;
                  
        reset_run(); // We restart from the begining

        // At this point the supervisor will have reset the robots.

        // Run the simulation for a while according to the info sent by the supervisor
        simulate(params);
    }
}

