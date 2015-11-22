#include <iostream>
#include <string>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

using namespace std;


// Fitness weights
static const float WEIGHT_ORIENTATION = 1.0;
static const float WEIGHT_COHESION = 1.0;
static const float WEIGHT_VELOCITY = 1.0;

// Warning: ADAPT TO THE NUMBER OF ROBOTS
static const int ROBOTS = 5;

static WbNodeRef robs[ROBOTS];
static WbFieldRef robs_translation[ROBOTS];
static WbFieldRef robs_rotation[ROBOTS];

double loc[ROBOTS][4]; // Contain the localisation of all robots

/*
 * Initialize flock position and devices.
 */
void reset(void)
{
  wb_robot_init(); // ??? <- Reset all epucks

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
}


/*
 * Compute orientaton metric.
 */
float getOrientation(void)
{
  return 0.0f;
}

/*
 * Compute cohesion metric.
 */
float getCohesion(void)
{
  return 0.0f;
}

/*
 * Compute velocity metric.
 */
float getVelocity(void)
{
  return 0.0f;
}

/*
 * Compute performance metric.
 */
float computeFitnessStep(void)
{
  return WEIGHT_ORIENTATION * getOrientation() +
         WEIGHT_COHESION * getCohesion() +
         WEIGHT_VELOCITY * getVelocity();
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
  while(true)
  {
    // Update the robot locations
    for (int i=0 ; i<ROBOTS ; i++)
    {
      loc[0][0] = wb_supervisor_field_get_sf_vec3f(robs_translation[0])[0];
      loc[0][1] = wb_supervisor_field_get_sf_vec3f(robs_translation[0])[1];
      loc[0][2] = wb_supervisor_field_get_sf_vec3f(robs_translation[0])[2];
      loc[0][3] = wb_supervisor_field_get_sf_rotation(robs_rotation[0])[3];
      loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[0];
      loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[1];
      loc[i][2] = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[2];
      loc[i][3] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3];
    }
    
    // Update fitness
    fitnessInstant = computeFitnessStep();
    fitnessGlobal += fitnessInstant;
    nbTimestep++;
    
    // Plot every 20 timesteps
    if(nbTimestep > 0 && nbTimestep % 20 == 0)
    {
      cout << "Performances:" << endl;
      cout << fitnessInstant << " (instant)" << endl;
      cout << fitnessGlobal/nbTimestep << " (global)" << endl;
    }
    
    // Limit
    if(nbTimestep > 2000)
    {
        break;
    }
    
    wb_robot_step(64); // ???
  }
  
  return 0;
}
