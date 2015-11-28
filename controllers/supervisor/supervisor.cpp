#include <iostream>
#include <string>
#include <complex>
#include <cmath>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

using namespace std;

//#define DEBUG_PRINT


// Fitness weights
static const float WEIGHT_ORIENTATION = 1.0;
static const float WEIGHT_COHESION = 1.0;
static const float WEIGHT_VELOCITY = 1.0;

// Warning: ADAPT TO THE NUMBER OF ROBOTS
static const int ROBOTS = 5;

static WbNodeRef robs[ROBOTS];
static WbFieldRef robs_translation[ROBOTS];
static WbFieldRef robs_rotation[ROBOTS];

float loc[ROBOTS][4]; // Contain the localisation of all robots
float centerOfMass[3] = {0.0f,0.0f,0.0f};
float prevCenterOfMass[3] = {0.0f,0.0f,0.0f}; // Location of the center of mass at time (t-1)
float migrationUrge[3] = {1.0f,0.0f,0.0f}; // Vector containing the direction of the urge


// -------------------
// Some math functions
// -------------------

/*
 * Compute the euclidian distance between two points
 */
float computeDist(float x[3],float y[3])
{
  return std::sqrt((x[0]-y[0]) * (x[0]-y[0]) +
                   (x[1]-y[1]) * (x[1]-y[1]) +
                   (x[2]-y[2]) * (x[2]-y[2]));
}

/*
 * Compute the dot product
 */
float computeDot(float x[3],float y[3])
{
  return x[0]*y[0] + x[1]*y[1] + x[2]*y[2];
}

// -------------------
// Fitness functions
// -------------------

/*
 * Compute orientaton metric.
 */
float getOrientation(void)
{
  std::complex<float> orientation = 0.0f;
  for(int i=0 ; i<ROBOTS ; ++i)
  {
    orientation += std::polar(1.0f, loc[i][3]); // 1.0 * exp(i*theta)
  }

  // Normalize
  return std::abs(orientation) / ROBOTS;
}

/*
 * Compute cohesion metric.
 */
float getCohesion(void)
{
  // The center of mass is already computed in the main fuction

  // Compute the average distance to the center of mass
  float dist = 0.0f;
  for(int i=0 ; i<ROBOTS ; ++i)
  {
    dist += computeDist(centerOfMass, loc[i]); // Thanks to the dirty C syntax, loc[i][4] is automatically "cast" into loc[i][3]
  }
  dist /= ROBOTS;

  // Return the metric
  return 1.0f/(1.0f+dist); // The closest, the better is
}

/*
 * Compute velocity metric.
 */
float getVelocity(void)
{
  const float maxVelocity = 0.001f; // TODO: Define max velocity of our webots

  // Compute the current velocity
  float velocityVector[3] = {centerOfMass[0] - prevCenterOfMass[0],
                             centerOfMass[1] - prevCenterOfMass[1],
                             centerOfMass[2] - prevCenterOfMass[2]}; // = x(t) - x(t-1)

  // First way: Compute the projection with respect to the migration urge
  //float currentVelocity = computeDot(velocityVector, migrationUrge);

  // Second way: We simply take the velocity of the center of mass
  float origin[3] = {0,0,0};
  float currentVelocity = computeDist(velocityVector, origin); // Norm of the vector

  return currentVelocity / maxVelocity; // Return the normalized velocity
}

/*
 * Compute performance metric.
 */
float computeFitnessStep(void)
{
  float orientation = getOrientation();
  float cohesion = getCohesion();
  float velocity = getVelocity();

#ifdef DEBUG_PRINT
  cout << "o:" << orientation << endl;
  cout << "c:" << cohesion << endl;
  cout << "v:" << velocity << endl;
#endif

  // Return normalized and weighted fitness
  return (WEIGHT_ORIENTATION * orientation +
          WEIGHT_COHESION * cohesion +
          WEIGHT_VELOCITY * velocity) /
         (WEIGHT_ORIENTATION + WEIGHT_COHESION + WEIGHT_VELOCITY); // Wrong: DO MULTIPLICATION INSTEAD OF ADDITION ?
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

  // Get the epucks from the tree
  for (int i=0 ; i<ROBOTS ; i++) {
    string robotName = "epuck" + std::to_string(i);

    robs[i] = wb_supervisor_node_get_from_def(robotName.c_str());
    if(robs[i] == NULL) // Check correctness
    {
      cout << "ERROR: Wrong number of epuck: " << robotName << endl;
      exit(-1);
    }
    robs_translation[i] = wb_supervisor_node_get_field(robs[i],"translation");
    robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
  }

  cout << ROBOTS << " robots loaded!" << endl;

  // TODO: Compute the migration urge
  // The migration urge is normalized
}

/*
 * Main function.
 */
int main(int argc, char *args[])
{
  cout << "Loading supervisor..." << endl;
  reset();

  float fitnessGlobal = 0.0f;
  float fitnessInstant = 0.0f;
  int nbTimestep = 0;

  // Main loop !
  bool finished = false;
  while(!finished)
  {
    // Update the robot locations
    for (int i=0 ; i<ROBOTS ; i++)
    {
      loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[0]; // x
      loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[1]; // y (probalby useless)
      loc[i][2] = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[2]; // z
      loc[i][3] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // theta (rotation)
    }

    // Update the center of mass
    for(int j=0 ; j<3 ; ++j) // 3 is the number of dimension
    {
      centerOfMass[j] = 0.0f; // Reset
    }
    for(int i=0 ; i<ROBOTS ; ++i)
    {
      for(int j=0 ; j<3 ; ++j)
      {
        centerOfMass[j] += loc[i][j] / ROBOTS; // We add the contribution of each robot
      }
    }

    // Update fitness
    if(nbTimestep > 0)
    {
      fitnessInstant = computeFitnessStep();
      fitnessGlobal += fitnessInstant;

      // Plot every x timesteps
      if(nbTimestep % 1000 == 0)
      {
        cout << "Performances:" << endl;
        cout << fitnessInstant << " (instant)" << endl;
        cout << fitnessGlobal/nbTimestep << " (global)" << endl;
      }
    }

    // (t-1) = t
    for(int j=0 ; j<3 ; ++j) // 3 is the number of dimension
    {
      prevCenterOfMass[j] = centerOfMass[j];
    }

    nbTimestep++;

    // Limit
    //if(nbTimestep > 2000)
    //{
    //    finished = true;
    //}

    wb_robot_step(64);
  }

  return 0;
}
