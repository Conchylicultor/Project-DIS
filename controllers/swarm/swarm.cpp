#include <iostream>
#include <cstdio> // For sscanf
#include <cmath>
#include <string>

#include <webots/robot.h>

#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

using namespace std;


// Global parametters
static const int FLOCK_SIZE = 5; // Number of robot in each flock (WARNING: Adapt to the number of robot)
static const int TIME_STEP = 64; // [ms] Length of time step

string robotName;
int robotIdGlobal = 0;
int robotId = 0; // Id for the flock

static const int NB_SENSORS = 8; // Number of distance sensors

WbDeviceTag ds[NB_SENSORS];	// Handle for the infrared distance sensors
WbDeviceTag receiver2;		// Handle for the receiver node
WbDeviceTag emitter2;		// Handle for the emitter node


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
    
    cout << "Loading: robot " << robotIdGlobal << " (" << (robotIdGlobal/FLOCK_SIZE) + 1 << ":" << robotId << ")" << endl;
    
    // Loading R&B module
    receiver2 = wb_robot_get_device("receiver");
    emitter2 = wb_robot_get_device("emitter");
    wb_receiver_enable(receiver2,64);
    
    // Loading the distances sensors
    for(int i=0; i<NB_SENSORS;i++) {
        ds[i]=wb_robot_get_device(string("ps" + std::to_string(i)).c_str());	// the device name is specified in the world file
        wb_distance_sensor_enable(ds[i],64);
    }
}

/*
 * Try avoiding obstacles
 */
void braitenbergObstacle(int wheelSpeed[2])
{
  // Parametters
  static int MIN_SENS = 350;
  static int weightMatrix[2][NB_SENSORS] = {{-72,-58,-36,8,10,36,28,18},
                                            {18,28,36,10,8,-36,-58,-72}}; // braitenberg weight
  int distance = 0;
  
  // Reinitialisation
  wheelSpeed[0] = 0;
  wheelSpeed[1] = 0;
  
  // Compute the ponderate behavior
  for(int i=0 ; i<NB_SENSORS ; i++)
  {
    distance = wb_distance_sensor_get_value(ds[i]); //Read sensor values
    
    // Weighted sum of distance sensor values for Braitenburg vehicle
    wheelSpeed[0] += weightMatrix[0][i] * distance; // Left
    wheelSpeed[1] += weightMatrix[1][i] * distance; // Right
  }
  
  // Adapt the speed
  wheelSpeed[0] /= MIN_SENS;
  wheelSpeed[1] /= MIN_SENS;
}

/*
 * Main function.
 */
int main(){ 
  // Initialize the robot
  reset();
  
  int wheelSpeed[2] = {0,0}; // Left and right wheel speed
  int wheelSpeedBraitenberg[2] = {0,0}; // Left and right wheel speed
  
  // Main loop
  while(true)
  {
    // Reinitialization
    
    // Braitenberg obstacle avoidance
    braitenbergObstacle(wheelSpeedBraitenberg);
    
    // Compute speed
    wheelSpeed[0] = 300;
    wheelSpeed[1] = 300;
    
    // Add Braitenberg
    wheelSpeed[0] += wheelSpeedBraitenberg[0];
    wheelSpeed[1] += wheelSpeedBraitenberg[1];
    
    // Set speed
    wb_differential_wheels_set_speed(wheelSpeed[0],wheelSpeed[1]);
    
    // Continue one step
    wb_robot_step(TIME_STEP);
  }
}  

